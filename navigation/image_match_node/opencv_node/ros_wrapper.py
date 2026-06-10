"""ROS 2 wrapper for classical OpenCV map-based localization."""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory

import cv2

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

from nav_msgs.msg import Odometry

import numpy as np

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image

from std_msgs.msg import Float32, Float64MultiArray, Int32

from tf2_ros import TransformBroadcaster

from opencv_node.chamfer import (
    centered_rigid_correction,
    chamfer_score,
    deterministic_subsample,
)
from opencv_node.lane_mask import (
    classify_lane_keypoints,
    extract_lane_mask,
    partition_features,
)
from opencv_node.map_geometry import pixel_to_world


class OpenCVLocalizationNode(Node):
    """Match BEV images to a map using local features and RANSAC."""

    def __init__(self):
        """Initialize the detector, map features, and ROS interfaces."""
        super().__init__('opencv_localization')
        self._declare_parameters()
        self._load_parameters()

        self._map_bgr = cv2.imread(self.map_image_path, cv2.IMREAD_COLOR)
        if self._map_bgr is None:
            raise FileNotFoundError(
                f'Failed to load map image: {self.map_image_path}')

        self._detector, norm_type = self._create_detector()
        self._matcher = cv2.BFMatcher(norm_type, crossCheck=False)
        map_gray = self._prepare_gray(self._map_bgr)
        self._map_keypoints, self._map_descriptors = (
            self._detector.detectAndCompute(map_gray, None))
        if self._map_descriptors is None or len(self._map_keypoints) < 4:
            raise RuntimeError(
                f'Not enough {self.feature_type.upper()} features in map: '
                f'{len(self._map_keypoints)}')

        self._map_lane_mask = np.zeros(
            self._map_bgr.shape[:2], dtype=np.uint8)
        self._map_lane_keypoints = []
        self._map_lane_descriptors = None
        self._map_non_lane_keypoints = self._map_keypoints
        self._map_non_lane_descriptors = self._map_descriptors
        self._map_lane_dt = None

        if self.enable_lane_features or self.enable_chamfer_refinement:
            self._map_lane_mask = self._extract_lane_mask(self._map_bgr)
        if self.enable_lane_features:
            map_lane_labels = classify_lane_keypoints(
                self._map_keypoints,
                self._map_lane_mask,
                self.lane_feature_dilation_ratio,
            )
            (
                self._map_lane_keypoints,
                self._map_lane_descriptors,
                self._map_non_lane_keypoints,
                self._map_non_lane_descriptors,
            ) = partition_features(
                self._map_keypoints,
                self._map_descriptors,
                map_lane_labels,
            )
            if self._map_lane_descriptors is None:
                self.get_logger().warning(
                    'Lane features enabled, but the map lane mask has no '
                    'classified descriptors. Non-lane features will be used.')
        if self.enable_chamfer_refinement:
            if not self._map_lane_mask.any():
                self.get_logger().warning(
                    'Chamfer refinement enabled but map lane mask is empty. '
                    'Refinement is disabled; check lane HSV parameters.')
            else:
                inv_mask = cv2.bitwise_not(self._map_lane_mask)
                self._map_lane_dt = cv2.distanceTransform(
                    inv_mask, cv2.DIST_L2, 5)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, sensor_qos)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.pose_topic, 10)
        self.odometry_pub = self.create_publisher(
            Odometry, self.odometry_topic, 10)
        self.debug_pub = self.create_publisher(
            Image, self.debug_topic, sensor_qos)
        self.homography_pub = self.create_publisher(
            Float64MultiArray, self.homography_topic, 10)
        self.match_count_pub = self.create_publisher(
            Int32, self.match_count_topic, 10)
        self.lane_match_count_pub = self.create_publisher(
            Int32, self.lane_match_count_topic, 10)
        self.lane_inlier_count_pub = self.create_publisher(
            Int32, self.lane_inlier_count_topic, 10)
        self.inlier_ratio_pub = self.create_publisher(
            Float32, self.inlier_ratio_topic, 10)
        self.chamfer_score_pub = self.create_publisher(
            Float32, self.chamfer_score_topic, 10)
        self.chamfer_refined_score_pub = self.create_publisher(
            Float32, self.chamfer_refined_score_topic, 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None)

        self._frame_count = 0
        self._busy = False
        self.get_logger().info(
            f'OpenCV {self.feature_type.upper()} localization ready: '
            f'map_features={len(self._map_keypoints)}, '
            f'map_lane_features={len(self._map_lane_keypoints)}, '
            f'homography={self.homography_method}, '
            f'chamfer_refinement={self._map_lane_dt is not None}, '
            f'image={self.image_topic}, map={self.map_image_path}')

    def _declare_parameters(self):
        default_map = os.path.join(
            get_package_share_directory('simple_gazebo_track'),
            'models', 'simple_track', 'materials', 'textures', 'track.png',
        )
        self.declare_parameter('image_topic', '/ipm_bev/fused')
        self.declare_parameter('map_image_path', default_map)

        self.declare_parameter('feature_type', 'sift')
        self.declare_parameter('max_features', 6000)
        self.declare_parameter('ratio_test', 0.72)
        self.declare_parameter('use_clahe', True)
        self.declare_parameter('frame_stride', 1)
        self.declare_parameter('min_matches', 12)
        self.declare_parameter('min_inliers', 8)
        self.declare_parameter('min_inlier_ratio', 0.25)
        self.declare_parameter('homography_method', 'usac_magsac')
        self.declare_parameter('ransac_reproj_threshold', 5.0)
        self.declare_parameter('forward_probe_distance', 1.0)

        self.declare_parameter('enable_lane_features', True)
        self.declare_parameter('lane_ratio_test', 0.75)
        self.declare_parameter('lane_max_matches', 100)
        self.declare_parameter('lane_hsv_lower', [0, 0, 165])
        self.declare_parameter('lane_hsv_upper', [179, 80, 255])
        self.declare_parameter('lane_roi_y_min_ratio', 0.0)
        self.declare_parameter('lane_roi_y_max_ratio', 1.0)
        self.declare_parameter('lane_kernel_size', 5)
        self.declare_parameter('lane_open_iterations', 0)
        self.declare_parameter('lane_close_iterations', 1)
        self.declare_parameter('lane_min_area', 20)
        self.declare_parameter('lane_kernel_size_ratio', 0.006)
        self.declare_parameter('lane_min_area_ratio', 0.00005)
        self.declare_parameter('lane_feature_dilation_ratio', 0.02)
        self.declare_parameter('enable_global_match_fallback', True)

        self.declare_parameter('enable_chamfer_refinement', False)
        self.declare_parameter('chamfer_max_pts', 400)
        self.declare_parameter('chamfer_score_gate', 20.0)
        self.declare_parameter('chamfer_min_coverage', 0.8)
        self.declare_parameter('chamfer_outside_penalty', 30.0)
        self.declare_parameter('chamfer_min_improvement', 0.25)
        self.declare_parameter('chamfer_max_translation_px', 20.0)
        self.declare_parameter('chamfer_max_rotation_deg', 5.0)
        self.declare_parameter(
            'chamfer_score_topic',
            '/visual_localization/opencv/chamfer_score')
        self.declare_parameter(
            'chamfer_refined_score_topic',
            '/visual_localization/opencv/chamfer_refined_score')

        self.declare_parameter('map_x_min', -6.0)
        self.declare_parameter('map_x_max', 6.0)
        self.declare_parameter('map_y_min', -8.0)
        self.declare_parameter('map_y_max', 8.0)
        self.declare_parameter('map_swap_xy', True)
        self.declare_parameter('map_flip_x', True)
        self.declare_parameter('map_flip_y', True)

        self.declare_parameter('bev_x_min', -10.0)
        self.declare_parameter('bev_x_max', 10.0)
        self.declare_parameter('bev_y_min', -6.0)
        self.declare_parameter('bev_y_max', 6.0)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('child_frame', 'base_footprint_opencv')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter(
            'pose_topic', '/visual_localization/opencv/pose')
        self.declare_parameter(
            'odometry_topic', '/visual_localization/opencv/odometry')
        self.declare_parameter(
            'debug_topic', '/visual_localization/opencv/debug_matches')
        self.declare_parameter(
            'homography_topic', '/visual_localization/opencv/homography')
        self.declare_parameter(
            'match_count_topic', '/visual_localization/opencv/match_count')
        self.declare_parameter(
            'lane_match_count_topic',
            '/visual_localization/opencv/lane_match_count')
        self.declare_parameter(
            'lane_inlier_count_topic',
            '/visual_localization/opencv/lane_inlier_count')
        self.declare_parameter(
            'inlier_ratio_topic', '/visual_localization/opencv/inlier_ratio')
        self.declare_parameter('max_debug_matches', 120)
        self.declare_parameter('debug_height', 720)

    def _load_parameters(self):
        self.image_topic = self.get_parameter('image_topic').value
        raw_map = self.get_parameter('map_image_path').value
        self.map_image_path = (
            os.path.expanduser(raw_map) if raw_map else os.path.join(
                get_package_share_directory('simple_gazebo_track'),
                'models', 'simple_track', 'materials', 'textures', 'track.png',
            ))

        self.feature_type = str(
            self.get_parameter('feature_type').value).lower()
        if self.feature_type not in ('sift', 'orb', 'akaze'):
            raise ValueError(
                'feature_type must be one of: sift, orb, akaze')
        self.max_features = max(
            100, int(self.get_parameter('max_features').value))
        self.ratio_test = float(self.get_parameter('ratio_test').value)
        if not 0.0 < self.ratio_test < 1.0:
            raise ValueError('ratio_test must be between 0 and 1.')
        self.use_clahe = bool(self.get_parameter('use_clahe').value)
        self.frame_stride = max(
            1, int(self.get_parameter('frame_stride').value))
        self.min_matches = max(
            4, int(self.get_parameter('min_matches').value))
        self.min_inliers = max(
            4, int(self.get_parameter('min_inliers').value))
        self.min_inlier_ratio = float(
            self.get_parameter('min_inlier_ratio').value)
        self.homography_method = str(
            self.get_parameter('homography_method').value).lower()
        if self.homography_method not in ('ransac', 'usac_magsac'):
            raise ValueError(
                'homography_method must be ransac or usac_magsac.')
        if (
                self.homography_method == 'usac_magsac'
                and not hasattr(cv2, 'USAC_MAGSAC')):
            self.get_logger().warning(
                'USAC_MAGSAC is unavailable in this OpenCV build. '
                'Falling back to RANSAC.')
            self.homography_method = 'ransac'
        self.ransac_reproj_threshold = float(
            self.get_parameter('ransac_reproj_threshold').value)
        self.forward_probe_distance = float(
            self.get_parameter('forward_probe_distance').value)

        self.enable_lane_features = bool(
            self.get_parameter('enable_lane_features').value)
        self.lane_ratio_test = float(
            self.get_parameter('lane_ratio_test').value)
        self.lane_max_matches = max(
            0, int(self.get_parameter('lane_max_matches').value))
        self.lane_hsv_lower = tuple(
            int(value) for value in
            self.get_parameter('lane_hsv_lower').value)
        self.lane_hsv_upper = tuple(
            int(value) for value in
            self.get_parameter('lane_hsv_upper').value)
        self.lane_roi_y_min_ratio = float(
            self.get_parameter('lane_roi_y_min_ratio').value)
        self.lane_roi_y_max_ratio = float(
            self.get_parameter('lane_roi_y_max_ratio').value)
        self.lane_kernel_size = max(
            1, int(self.get_parameter('lane_kernel_size').value))
        if self.lane_kernel_size % 2 == 0:
            self.lane_kernel_size += 1
        self.lane_open_iterations = max(
            0, int(self.get_parameter('lane_open_iterations').value))
        self.lane_close_iterations = max(
            0, int(self.get_parameter('lane_close_iterations').value))
        self.lane_min_area = max(
            0, int(self.get_parameter('lane_min_area').value))
        self.lane_kernel_size_ratio = max(
            0.0,
            float(self.get_parameter('lane_kernel_size_ratio').value),
        )
        self.lane_min_area_ratio = max(
            0.0,
            float(self.get_parameter('lane_min_area_ratio').value),
        )
        self.lane_feature_dilation_ratio = max(
            0.0,
            float(self.get_parameter('lane_feature_dilation_ratio').value),
        )
        self.enable_global_match_fallback = bool(
            self.get_parameter('enable_global_match_fallback').value)

        self.enable_chamfer_refinement = bool(
            self.get_parameter('enable_chamfer_refinement').value)
        self.chamfer_max_pts = max(
            10, int(self.get_parameter('chamfer_max_pts').value))
        self.chamfer_score_gate = float(
            self.get_parameter('chamfer_score_gate').value)
        self.chamfer_min_coverage = float(
            self.get_parameter('chamfer_min_coverage').value)
        self.chamfer_outside_penalty = float(
            self.get_parameter('chamfer_outside_penalty').value)
        self.chamfer_min_improvement = float(
            self.get_parameter('chamfer_min_improvement').value)
        self.chamfer_max_translation_px = float(
            self.get_parameter('chamfer_max_translation_px').value)
        self.chamfer_max_rotation_rad = math.radians(float(
            self.get_parameter('chamfer_max_rotation_deg').value))
        self.chamfer_score_topic = self.get_parameter(
            'chamfer_score_topic').value
        self.chamfer_refined_score_topic = self.get_parameter(
            'chamfer_refined_score_topic').value

        self.map_x_min = float(self.get_parameter('map_x_min').value)
        self.map_x_max = float(self.get_parameter('map_x_max').value)
        self.map_y_min = float(self.get_parameter('map_y_min').value)
        self.map_y_max = float(self.get_parameter('map_y_max').value)
        self.map_swap_xy = bool(self.get_parameter('map_swap_xy').value)
        self.map_flip_x = bool(self.get_parameter('map_flip_x').value)
        self.map_flip_y = bool(self.get_parameter('map_flip_y').value)
        self.bev_x_min = float(self.get_parameter('bev_x_min').value)
        self.bev_x_max = float(self.get_parameter('bev_x_max').value)
        self.bev_y_min = float(self.get_parameter('bev_y_min').value)
        self.bev_y_max = float(self.get_parameter('bev_y_max').value)

        self.map_frame = self.get_parameter('map_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.homography_topic = self.get_parameter('homography_topic').value
        self.match_count_topic = self.get_parameter('match_count_topic').value
        self.lane_match_count_topic = self.get_parameter(
            'lane_match_count_topic').value
        self.lane_inlier_count_topic = self.get_parameter(
            'lane_inlier_count_topic').value
        self.inlier_ratio_topic = self.get_parameter(
            'inlier_ratio_topic').value
        self.max_debug_matches = int(
            self.get_parameter('max_debug_matches').value)
        self.debug_height = int(self.get_parameter('debug_height').value)

        if (
                self.map_x_max <= self.map_x_min
                or self.map_y_max <= self.map_y_min):
            raise ValueError('Invalid map bounds.')
        if (
                self.bev_x_max <= self.bev_x_min
                or self.bev_y_max <= self.bev_y_min):
            raise ValueError('Invalid BEV bounds.')
        if not 0.0 < self.lane_ratio_test < 1.0:
            raise ValueError('lane_ratio_test must be between 0 and 1.')
        if len(self.lane_hsv_lower) != 3 or len(self.lane_hsv_upper) != 3:
            raise ValueError('lane HSV bounds must contain three integers.')
        if not (
                0.0 <= self.lane_roi_y_min_ratio
                <= self.lane_roi_y_max_ratio <= 1.0):
            raise ValueError('Invalid lane ROI ratios.')
        if not 0.0 < self.chamfer_min_coverage <= 1.0:
            raise ValueError(
                'chamfer_min_coverage must be in the range (0, 1].')
        if self.chamfer_outside_penalty < 0.0:
            raise ValueError('chamfer_outside_penalty must be non-negative.')

    def _create_detector(self):
        if self.feature_type == 'sift':
            return cv2.SIFT_create(nfeatures=self.max_features), cv2.NORM_L2
        if self.feature_type == 'orb':
            return (
                cv2.ORB_create(
                    nfeatures=self.max_features,
                    scaleFactor=1.2,
                    nlevels=8,
                    fastThreshold=12,
                ),
                cv2.NORM_HAMMING,
            )
        return cv2.AKAZE_create(), cv2.NORM_HAMMING

    def _prepare_gray(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.use_clahe:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
        return gray

    def _extract_lane_mask(self, image):
        return extract_lane_mask(
            image,
            self.lane_hsv_lower,
            self.lane_hsv_upper,
            self.lane_roi_y_min_ratio,
            self.lane_roi_y_max_ratio,
            self.lane_kernel_size,
            self.lane_open_iterations,
            self.lane_close_iterations,
            self.lane_min_area,
            self.lane_kernel_size_ratio,
            self.lane_min_area_ratio,
        )

    def _chamfer_score(self, homography, pts_xy):
        """Return penalized Chamfer score and in-map point coverage."""
        return chamfer_score(
            self._map_lane_dt,
            homography,
            pts_xy,
            self.chamfer_min_coverage,
            self.chamfer_outside_penalty,
        )

    def _refine_with_chamfer(self, homography, bev_lane_mask):
        """Refine homography with coarse-to-fine coordinate descent on EDT.

        Returns (refined_H, score_before, score_after).
        """
        pts = np.argwhere(bev_lane_mask > 0)
        if len(pts) < 10:
            return homography, float('inf'), float('inf'), 0.0, 0.0
        pts = deterministic_subsample(pts, self.chamfer_max_pts)
        pts_xy = pts[:, ::-1].astype(np.float32).reshape(-1, 1, 2)

        score_before, coverage_before = self._chamfer_score(
            homography, pts_xy)
        if (
                not math.isfinite(score_before)
                or score_before > self.chamfer_score_gate):
            return (
                homography,
                score_before,
                score_before,
                coverage_before,
                coverage_before,
            )

        bev_center = self._bev_metric_to_pixel(
            0.0,
            0.0,
            bev_lane_mask.shape[1],
            bev_lane_mask.shape[0],
        )
        correction_center = cv2.perspectiveTransform(
            np.asarray([[bev_center]], dtype=np.float32),
            homography,
        )[0, 0]
        if (
                not np.isfinite(correction_center).all()
                or not self._inside_map(correction_center)):
            return (
                homography,
                score_before,
                score_before,
                coverage_before,
                coverage_before,
            )

        best_params = np.zeros(3)
        best_score = score_before
        best_coverage = coverage_before

        for scale in (4.0, 1.5, 0.5):
            steps = np.array([scale, scale, scale * 0.008])
            for _ in range(12):
                moved = False
                for axis in range(3):
                    for sign in (1.0, -1.0):
                        trial = best_params.copy()
                        trial[axis] += sign * steps[axis]
                        if (
                                np.hypot(trial[0], trial[1])
                                > self.chamfer_max_translation_px
                                or abs(trial[2])
                                > self.chamfer_max_rotation_rad):
                            continue
                        correction = centered_rigid_correction(
                            trial, correction_center)
                        score, coverage = self._chamfer_score(
                            correction @ homography, pts_xy)
                        if score < best_score - 0.05:
                            best_score = score
                            best_coverage = coverage
                            best_params = trial
                            moved = True
                            break
                if not moved:
                    break

        if best_score <= score_before - self.chamfer_min_improvement:
            correction = centered_rigid_correction(
                best_params, correction_center)
            return (
                correction @ homography,
                score_before,
                best_score,
                coverage_before,
                best_coverage,
            )
        return (
            homography,
            score_before,
            score_before,
            coverage_before,
            coverage_before,
        )

    def _match_features(
            self, query_keypoints, query_descriptors,
            map_keypoints, map_descriptors, ratio_test, max_matches=0):
        if (
                query_descriptors is None or map_descriptors is None
                or len(query_keypoints) < 2 or len(map_keypoints) < 2):
            return (
                np.empty((0, 2), dtype=np.float32),
                np.empty((0, 2), dtype=np.float32),
                np.empty((0,), dtype=np.float32),
            )

        pairs = self._matcher.knnMatch(
            query_descriptors, map_descriptors, k=2)
        good_matches = [
            first for first, second in pairs
            if first.distance < ratio_test * second.distance
        ]
        best_by_map_feature = {}
        for match in good_matches:
            previous = best_by_map_feature.get(match.trainIdx)
            if previous is None or match.distance < previous.distance:
                best_by_map_feature[match.trainIdx] = match
        good_matches = list(best_by_map_feature.values())
        if max_matches > 0 and len(good_matches) > max_matches:
            good_matches = sorted(
                good_matches, key=lambda match: match.distance)[:max_matches]

        query_points = np.float32(
            [query_keypoints[match.queryIdx].pt for match in good_matches]
        ).reshape(-1, 2)
        map_points = np.float32(
            [map_keypoints[match.trainIdx].pt for match in good_matches]
        ).reshape(-1, 2)
        distances = np.float32(
            [match.distance for match in good_matches])
        return query_points, map_points, distances

    def _image_callback(self, msg):
        self._frame_count += 1
        if self._busy or self._frame_count % self.frame_stride != 0:
            return
        self._busy = True
        try:
            self._localize(self._ros_image_to_bgr(msg), msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'OpenCV localization failed: {type(exc).__name__}: {exc}')
        finally:
            self._busy = False

    def _localize(self, bev_bgr, source_msg):
        start = time.perf_counter()
        bev_gray = self._prepare_gray(bev_bgr)
        bev_keypoints, bev_descriptors = self._detector.detectAndCompute(
            bev_gray, None)

        bev_lane_mask = np.zeros(bev_bgr.shape[:2], dtype=np.uint8)
        lane_bev_points = np.empty((0, 2), dtype=np.float32)
        lane_map_points = np.empty((0, 2), dtype=np.float32)
        lane_distances = np.empty((0,), dtype=np.float32)
        lane_keypoint_count = 0
        if self.enable_lane_features or self.enable_chamfer_refinement:
            bev_lane_mask = self._extract_lane_mask(bev_bgr)
        if self.enable_lane_features:
            lane_labels = classify_lane_keypoints(
                bev_keypoints,
                bev_lane_mask,
                self.lane_feature_dilation_ratio,
            )
            (
                lane_keypoints,
                lane_descriptors,
                non_lane_keypoints,
                non_lane_descriptors,
            ) = partition_features(
                bev_keypoints,
                bev_descriptors,
                lane_labels,
            )
            lane_keypoint_count = len(lane_keypoints)
            raw_bev_points, raw_map_points, raw_distances = (
                self._match_features(
                    non_lane_keypoints,
                    non_lane_descriptors,
                    self._map_non_lane_keypoints,
                    self._map_non_lane_descriptors,
                    self.ratio_test,
                ))
            lane_bev_points, lane_map_points, lane_distances = (
                self._match_features(
                    lane_keypoints,
                    lane_descriptors,
                    self._map_lane_keypoints,
                    self._map_lane_descriptors,
                    self.lane_ratio_test,
                    self.lane_max_matches,
                ))
        else:
            raw_bev_points, raw_map_points, raw_distances = (
                self._match_features(
                    bev_keypoints,
                    bev_descriptors,
                    self._map_keypoints,
                    self._map_descriptors,
                    self.ratio_test,
                ))

        used_global_fallback = False
        semantic_match_count = len(raw_bev_points) + len(lane_bev_points)
        if (
                self.enable_lane_features
                and self.enable_global_match_fallback
                and semantic_match_count < self.min_matches):
            raw_bev_points, raw_map_points, raw_distances = (
                self._match_features(
                    bev_keypoints,
                    bev_descriptors,
                    self._map_keypoints,
                    self._map_descriptors,
                    self.ratio_test,
                ))
            lane_bev_points = np.empty((0, 2), dtype=np.float32)
            lane_map_points = np.empty((0, 2), dtype=np.float32)
            lane_distances = np.empty((0,), dtype=np.float32)
            used_global_fallback = True

        raw_match_count = len(raw_bev_points)
        lane_match_count = len(lane_bev_points)
        total_match_count = raw_match_count + lane_match_count
        self.match_count_pub.publish(Int32(data=total_match_count))
        self.lane_match_count_pub.publish(Int32(data=lane_match_count))
        self.lane_inlier_count_pub.publish(Int32(data=0))
        if total_match_count < self.min_matches:
            self.get_logger().warning(
                f'Rejected: non_lane={raw_match_count}, '
                f'lane={lane_match_count}, '
                f'semantic={semantic_match_count}, '
                f'total={total_match_count} ratio-test matches '
                f'< {self.min_matches}.')
            return

        bev_points = np.concatenate(
            (raw_bev_points, lane_bev_points), axis=0)
        map_points = np.concatenate(
            (raw_map_points, lane_map_points), axis=0)
        distances = np.concatenate(
            (raw_distances, lane_distances), axis=0)
        lane_sources = np.concatenate((
            np.zeros(raw_match_count, dtype=bool),
            np.ones(lane_match_count, dtype=bool),
        ))
        scores = 1.0 / (1.0 + distances)

        method = (
            cv2.USAC_MAGSAC
            if self.homography_method == 'usac_magsac'
            else cv2.RANSAC
        )
        homography, inlier_mask = cv2.findHomography(
            bev_points,
            map_points,
            method,
            self.ransac_reproj_threshold,
        )
        if homography is None or inlier_mask is None:
            self.get_logger().warning(
                'Rejected: homography estimation failed.')
            return

        inlier_mask = inlier_mask.ravel().astype(bool)
        inlier_count = int(np.count_nonzero(inlier_mask))
        lane_inlier_count = int(np.count_nonzero(
            np.logical_and(inlier_mask, lane_sources)))
        self.lane_inlier_count_pub.publish(
            Int32(data=lane_inlier_count))
        inlier_ratio = inlier_count / max(1, len(inlier_mask))
        self.inlier_ratio_pub.publish(Float32(data=float(inlier_ratio)))
        if (
                inlier_count < self.min_inliers
                or inlier_ratio < self.min_inlier_ratio):
            self.get_logger().warning(
                f'Rejected: matches={total_match_count}, '
                f'inliers={inlier_count}, ratio={inlier_ratio:.3f}.')
            self._publish_debug(
                bev_bgr, map_points, bev_points, scores, inlier_mask,
                lane_sources, bev_lane_mask, None, source_msg)
            return

        chamfer_before = float('inf')
        chamfer_after = float('inf')
        chamfer_coverage_before = 0.0
        chamfer_coverage_after = 0.0
        if self.enable_chamfer_refinement and self._map_lane_dt is not None:
            (
                homography,
                chamfer_before,
                chamfer_after,
                chamfer_coverage_before,
                chamfer_coverage_after,
            ) = (
                self._refine_with_chamfer(homography, bev_lane_mask))
        self.chamfer_score_pub.publish(Float32(data=float(chamfer_before)))
        self.chamfer_refined_score_pub.publish(
            Float32(data=float(chamfer_after)))

        bev_center = self._bev_metric_to_pixel(
            0.0, 0.0, bev_bgr.shape[1], bev_bgr.shape[0])
        bev_forward = self._bev_metric_to_pixel(
            self.forward_probe_distance, 0.0,
            bev_bgr.shape[1], bev_bgr.shape[0])
        projected = cv2.perspectiveTransform(
            np.array([[bev_center, bev_forward]], dtype=np.float32),
            homography,
        )[0]
        center_uv, forward_uv = projected
        if not self._inside_map(center_uv) or not self._inside_map(forward_uv):
            self.get_logger().warning(
                'Rejected: projected vehicle points outside map.')
            return

        x, y = self._map_pixel_to_world(center_uv)
        forward_x, forward_y = self._map_pixel_to_world(forward_uv)
        yaw = math.atan2(forward_y - y, forward_x - x)

        pose_msg = self._make_pose(source_msg, x, y, yaw, inlier_ratio)
        self.pose_pub.publish(pose_msg)
        self.odometry_pub.publish(self._make_odometry(pose_msg))
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                self._make_transform(source_msg, x, y, yaw))

        homography_msg = Float64MultiArray()
        homography_msg.data = homography.reshape(-1).astype(float).tolist()
        self.homography_pub.publish(homography_msg)
        self._publish_debug(
            bev_bgr, map_points, bev_points, scores, inlier_mask,
            lane_sources, bev_lane_mask, center_uv, source_msg)

        elapsed = time.perf_counter() - start
        chamfer_str = (
            f' chamfer={chamfer_before:.1f}->{chamfer_after:.1f}'
            f' coverage={chamfer_coverage_before:.2f}'
            f'->{chamfer_coverage_after:.2f}'
            if math.isfinite(chamfer_before) else ''
        )
        self.get_logger().info(
            f'pose=({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}deg), '
            f'features={len(bev_keypoints)} lane={lane_keypoint_count}, '
            f'matches={raw_match_count}+{lane_match_count}, '
            f'inliers={inlier_count} lane={lane_inlier_count} '
            f'fallback={used_global_fallback} '
            f'({inlier_ratio:.2f}),{chamfer_str} '
            f'time={elapsed:.3f}s')

    def _bev_metric_to_pixel(self, x, y, width, height):
        u = (self.bev_y_max - y) / (
            self.bev_y_max - self.bev_y_min) * (width - 1)
        v = (self.bev_x_max - x) / (
            self.bev_x_max - self.bev_x_min) * (height - 1)
        return [float(u), float(v)]

    def _map_pixel_to_world(self, point):
        return pixel_to_world(
            point,
            self._map_bgr.shape,
            (self.map_x_min, self.map_x_max),
            (self.map_y_min, self.map_y_max),
            self.map_swap_xy,
            self.map_flip_x,
            self.map_flip_y,
        )

    def _inside_map(self, point):
        return (
            np.isfinite(point).all()
            and 0.0 <= point[0] < self._map_bgr.shape[1]
            and 0.0 <= point[1] < self._map_bgr.shape[0]
        )

    def _make_pose(self, source_msg, x, y, yaw, inlier_ratio):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = source_msg.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        position_variance = max(0.01, (1.0 - inlier_ratio) ** 2)
        yaw_variance = max(0.01, (1.0 - inlier_ratio) * 0.5)
        msg.pose.covariance[0] = position_variance
        msg.pose.covariance[7] = position_variance
        msg.pose.covariance[14] = 1.0
        msg.pose.covariance[21] = 1.0
        msg.pose.covariance[28] = 1.0
        msg.pose.covariance[35] = yaw_variance
        return msg

    def _make_transform(self, source_msg, x, y, yaw):
        msg = TransformStamped()
        msg.header.stamp = source_msg.header.stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.child_frame
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.rotation.z = math.sin(yaw * 0.5)
        msg.transform.rotation.w = math.cos(yaw * 0.5)
        return msg

    def _make_odometry(self, pose_msg):
        msg = Odometry()
        msg.header = pose_msg.header
        msg.child_frame_id = self.child_frame
        msg.pose = pose_msg.pose
        for index in (0, 7, 14, 21, 28, 35):
            msg.twist.covariance[index] = 1.0e6
        return msg

    def _publish_debug(
            self, bev_bgr, map_points, bev_points, scores, inlier_mask,
            lane_sources, bev_lane_mask, center_uv, source_msg):
        map_view = self._map_bgr.copy()
        bev_view = bev_bgr.copy()
        self._tint_mask(map_view, self._map_lane_mask, (0, 180, 255))
        self._tint_mask(bev_view, bev_lane_mask, (0, 180, 255))
        if center_uv is not None:
            cv2.drawMarker(
                map_view,
                tuple(np.round(center_uv).astype(int)),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                36,
                4,
            )

        target_height = max(240, self.debug_height)
        map_scale = target_height / map_view.shape[0]
        bev_scale = target_height / bev_bgr.shape[0]
        map_resized = cv2.resize(
            map_view, None, fx=map_scale, fy=map_scale,
            interpolation=cv2.INTER_AREA)
        bev_resized = cv2.resize(
            bev_view, None, fx=bev_scale, fy=bev_scale,
            interpolation=cv2.INTER_AREA)
        canvas = np.zeros(
            (
                target_height,
                map_resized.shape[1] + bev_resized.shape[1],
                3,
            ),
            dtype=np.uint8,
        )
        canvas[:, :map_resized.shape[1]] = map_resized
        canvas[:, map_resized.shape[1]:] = bev_resized

        indices = np.flatnonzero(inlier_mask)
        if len(indices) > self.max_debug_matches:
            ranked = indices[np.argsort(scores[indices])]
            indices = ranked[-self.max_debug_matches:]
        offset = map_resized.shape[1]
        for index in indices:
            line_color = (
                (255, 80, 255) if lane_sources[index]
                else (0, 220, 0))
            p0 = tuple(np.round(
                map_points[index] * map_scale).astype(int))
            p1_local = np.round(
                bev_points[index] * bev_scale).astype(int)
            p1 = (int(p1_local[0] + offset), int(p1_local[1]))
            cv2.line(canvas, p0, p1, line_color, 1, cv2.LINE_AA)
            cv2.circle(canvas, p0, 2, line_color, -1)
            cv2.circle(canvas, p1, 2, line_color, -1)

        self.debug_pub.publish(self._bgr_to_ros_image(canvas, source_msg))

    @staticmethod
    def _tint_mask(image, mask, color):
        pixels = mask > 0
        if not pixels.any():
            return
        image[pixels] = (
            image[pixels].astype(np.float32) * 0.7
            + np.asarray(color, dtype=np.float32) * 0.3
        ).astype(np.uint8)

    @staticmethod
    def _ros_image_to_bgr(msg):
        encoding = msg.encoding.lower()
        channel_map = {
            'mono8': 1,
            '8uc1': 1,
            'bgr8': 3,
            'rgb8': 3,
            'bgra8': 4,
            'rgba8': 4,
        }
        if encoding not in channel_map:
            raise ValueError(f'Unsupported image encoding: {msg.encoding}')
        channels = channel_map[encoding]
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.step)
        if channels > 1:
            image = rows[:, :msg.width * channels].reshape(
                msg.height, msg.width, channels).copy()
        else:
            image = rows[:, :msg.width].reshape(
                msg.height, msg.width).copy()
        if encoding in ('mono8', '8uc1'):
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    @staticmethod
    def _bgr_to_ros_image(image, source_msg):
        msg = Image()
        msg.header = source_msg.header
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(image.shape[1] * 3)
        msg.data = np.ascontiguousarray(image).tobytes()
        return msg
