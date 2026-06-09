"""ROS 2 wrapper for MINIMA map-based visual localization."""

import math
import os
import sys
import time
import types
from pathlib import Path

import cv2
import numpy as np
import torch

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64MultiArray, Int32

from tf2_ros import TransformBroadcaster


def _resolve_minima_root() -> Path:
    # 1. 환경변수가 설정된 경우 최우선
    env = os.environ.get('MINIMA_ROOT')
    if env:
        return Path(env)
    # 2. 소스 트리 내 경로 (colcon build --symlink-install 또는 개발 환경)
    candidate = Path(__file__).resolve().parents[1] / 'MINIMA'
    if candidate.is_dir():
        return candidate
    # 3. 워크스페이스 절대 경로 fallback
    return Path('/ros_ws/src/aac_2026/navigation/image_match_node/MINIMA')


MINIMA_ROOT = _resolve_minima_root()

_WEIGHT_DEFAULTS = {
    'loftr': 'minima_loftr.ckpt',
    'roma':  'minima_roma.pth',
    'sp_lg': 'minima_lightglue.pth',
}

MATCH_COLORS_BGR = np.array(
    [
        (32,  64, 255),
        (255, 180,  64),
        (90, 220, 120),
        (64, 220, 255),
        (220,  80, 220),
        (200, 220,  64),
        (40, 140, 255),
        (255, 120, 160),
    ],
    dtype=np.uint8,
)


def _load_minima_matcher(method: str, ckpt_path: str, match_thr: float):
    """Load a MINIMA matcher and return its from_cv_imgs callable."""
    for p in (str(MINIMA_ROOT), str(MINIMA_ROOT / 'third_party' / 'RoMa_minima')):
        if p not in sys.path:
            sys.path.insert(0, p)

    prev_dir = os.getcwd()
    os.chdir(MINIMA_ROOT)
    try:
        from load_model import load_model  # noqa: PLC0415
        args = types.SimpleNamespace()
        args.ckpt = ckpt_path
        if method == 'loftr':
            args.thr = match_thr
        elif method == 'roma':
            args.ckpt2 = 'large'
        return load_model(method, args, use_path=False)
    finally:
        os.chdir(prev_dir)


class MINIMALocalizationNode(Node):
    """Match a vehicle-centered BEV image against a global map using MINIMA."""

    def __init__(self):
        """Initialize map, model, ROS interfaces, and parameters."""
        super().__init__('minima_localization')
        self._declare_parameters()
        self._load_parameters()

        self._map_bgr = cv2.imread(self.map_image_path, cv2.IMREAD_COLOR)
        if self._map_bgr is None:
            raise FileNotFoundError(
                f'Failed to load map image: {self.map_image_path}')

        self.get_logger().info(
            f'Loading MINIMA [{self.method}] from {self.ckpt_path} ...')
        self._matcher = _load_minima_matcher(
            self.method, self.ckpt_path, self.match_thr)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, sensor_qos)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.pose_topic, 10)
        self.debug_pub = self.create_publisher(
            Image, self.debug_topic, sensor_qos)
        self.homography_pub = self.create_publisher(
            Float64MultiArray, self.homography_topic, 10)
        self.match_count_pub = self.create_publisher(
            Int32, self.match_count_topic, 10)
        self.inlier_ratio_pub = self.create_publisher(
            Float32, self.inlier_ratio_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._frame_count = 0
        self._busy = False
        self.get_logger().info(
            f'MINIMA [{self.method}] localization ready: '
            f'image={self.image_topic}, '
            f'map={self.map_image_path} '
            f'({self._map_bgr.shape[1]}x{self._map_bgr.shape[0]})')

    # ------------------------------------------------------------------
    # Parameter declaration / loading
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        default_map = os.path.join(
            get_package_share_directory('simple_gazebo_track'),
            'models', 'simple_track', 'materials', 'textures', 'track.png',
        )
        self.declare_parameter('image_topic', '/ipm_bev/fused')
        self.declare_parameter('map_image_path', default_map)

        self.declare_parameter('method', 'loftr')
        self.declare_parameter('ckpt_path', '')
        self.declare_parameter('match_thr', 0.2)
        self.declare_parameter('device', 'cuda')

        self.declare_parameter('frame_stride', 3)
        self.declare_parameter('min_matches', 40)
        self.declare_parameter('min_inliers', 20)
        self.declare_parameter('min_inlier_ratio', 0.2)
        self.declare_parameter('ransac_reproj_threshold', 4.0)
        self.declare_parameter('forward_probe_distance', 1.0)

        self.declare_parameter('map_x_min', -6.0)
        self.declare_parameter('map_x_max', 6.0)
        self.declare_parameter('map_y_min', -8.0)
        self.declare_parameter('map_y_max', 8.0)
        self.declare_parameter('map_swap_xy', True)
        self.declare_parameter('map_flip_x', True)
        self.declare_parameter('map_flip_y', False)

        self.declare_parameter('bev_x_min', -10.0)
        self.declare_parameter('bev_x_max', 10.0)
        self.declare_parameter('bev_y_min', -6.0)
        self.declare_parameter('bev_y_max', 6.0)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('pose_topic', '/visual_localization/pose')
        self.declare_parameter('debug_topic', '/visual_localization/debug_matches')
        self.declare_parameter('homography_topic', '/visual_localization/homography')
        self.declare_parameter('match_count_topic', '/visual_localization/match_count')
        self.declare_parameter('inlier_ratio_topic', '/visual_localization/inlier_ratio')
        self.declare_parameter('max_debug_matches', 120)
        self.declare_parameter('debug_height', 720)

    def _load_parameters(self):
        self.image_topic = self.get_parameter('image_topic').value

        raw_map = self.get_parameter('map_image_path').value
        self.map_image_path = os.path.expanduser(raw_map) if raw_map else os.path.join(
            get_package_share_directory('simple_gazebo_track'),
            'models', 'simple_track', 'materials', 'textures', 'track.png',
        )

        self.method = self.get_parameter('method').value
        if self.method not in _WEIGHT_DEFAULTS:
            raise ValueError(
                f"Unknown MINIMA method '{self.method}'. "
                f"Choose from: {list(_WEIGHT_DEFAULTS)}")

        raw_ckpt = self.get_parameter('ckpt_path').value
        if raw_ckpt:
            checkpoint = Path(os.path.expanduser(raw_ckpt))
            if not checkpoint.is_absolute():
                checkpoint = MINIMA_ROOT / 'weights' / checkpoint
        else:
            checkpoint = MINIMA_ROOT / 'weights' / _WEIGHT_DEFAULTS[self.method]

        checkpoint = checkpoint.resolve()
        if not checkpoint.is_file():
            raise FileNotFoundError(
                f"MINIMA checkpoint for method '{self.method}' was not found: "
                f'{checkpoint}. Set ckpt_path to an existing file, or leave it '
                'empty to select the method default.')
        self.ckpt_path = str(checkpoint)

        self.match_thr = float(self.get_parameter('match_thr').value)

        requested_device = self.get_parameter('device').value
        if requested_device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warning('CUDA unavailable; falling back to CPU.')
            requested_device = 'cpu'
        self.device = requested_device

        self.frame_stride = max(1, int(self.get_parameter('frame_stride').value))
        self.min_matches = int(self.get_parameter('min_matches').value)
        self.min_inliers = int(self.get_parameter('min_inliers').value)
        self.min_inlier_ratio = float(self.get_parameter('min_inlier_ratio').value)
        self.ransac_reproj_threshold = float(
            self.get_parameter('ransac_reproj_threshold').value)
        self.forward_probe_distance = float(
            self.get_parameter('forward_probe_distance').value)

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

        if self.map_x_max <= self.map_x_min or self.map_y_max <= self.map_y_min:
            raise ValueError('Invalid map bounds.')
        if self.bev_x_max <= self.bev_x_min or self.bev_y_max <= self.bev_y_min:
            raise ValueError('Invalid BEV bounds.')

        self.map_frame = self.get_parameter('map_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.homography_topic = self.get_parameter('homography_topic').value
        self.match_count_topic = self.get_parameter('match_count_topic').value
        self.inlier_ratio_topic = self.get_parameter('inlier_ratio_topic').value
        self.max_debug_matches = int(self.get_parameter('max_debug_matches').value)
        self.debug_height = int(self.get_parameter('debug_height').value)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _image_callback(self, msg):
        self._frame_count += 1
        if self._busy or self._frame_count % self.frame_stride != 0:
            return

        self._busy = True
        try:
            bev_bgr = self._ros_image_to_bgr(msg)
            self._localize(bev_bgr, msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'MINIMA localization failed: {type(exc).__name__}: {exc}')
        finally:
            self._busy = False

    # ------------------------------------------------------------------
    # Localization pipeline
    # ------------------------------------------------------------------

    def _localize(self, bev_bgr, source_msg):
        start = time.perf_counter()

        result = self._matcher(self._map_bgr, bev_bgr)
        map_points = result['mkpts0'].astype(np.float32)
        bev_points = result['mkpts1'].astype(np.float32)
        scores = result['mconf'].astype(np.float32)

        self.match_count_pub.publish(Int32(data=int(len(map_points))))

        if len(map_points) < self.min_matches:
            self.get_logger().warning(
                f'Rejected: {len(map_points)} matches < {self.min_matches}.')
            return

        homography, inlier_mask = cv2.findHomography(
            bev_points, map_points, cv2.RANSAC, self.ransac_reproj_threshold)
        if homography is None or inlier_mask is None:
            self.get_logger().warning('Rejected: homography estimation failed.')
            return

        inlier_mask = inlier_mask.ravel().astype(bool)
        inlier_count = int(np.count_nonzero(inlier_mask))
        inlier_ratio = inlier_count / max(1, len(inlier_mask))
        self.inlier_ratio_pub.publish(Float32(data=float(inlier_ratio)))

        if inlier_count < self.min_inliers or inlier_ratio < self.min_inlier_ratio:
            self.get_logger().warning(
                f'Rejected: inliers={inlier_count}, ratio={inlier_ratio:.3f}.')
            self._publish_debug(
                bev_bgr, map_points, bev_points, scores, inlier_mask,
                None, source_msg)
            return

        bev_center = self._bev_metric_to_pixel(
            0.0, 0.0, bev_bgr.shape[1], bev_bgr.shape[0])
        bev_forward = self._bev_metric_to_pixel(
            self.forward_probe_distance, 0.0,
            bev_bgr.shape[1], bev_bgr.shape[0])
        query_pts = np.array([[bev_center, bev_forward]], dtype=np.float32)
        projected = cv2.perspectiveTransform(query_pts, homography)[0]
        center_uv, forward_uv = projected

        if not self._inside_map(center_uv):
            self.get_logger().warning(
                f'Rejected: projected center outside map '
                f'({center_uv[0]:.1f}, {center_uv[1]:.1f}).')
            return

        x, y = self._map_pixel_to_world(center_uv)
        fx, fy = self._map_pixel_to_world(forward_uv)
        yaw = math.atan2(fy - y, fx - x)

        self.pose_pub.publish(self._make_pose(source_msg, x, y, yaw, inlier_ratio))
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                self._make_transform(source_msg, x, y, yaw))

        homography_msg = Float64MultiArray()
        homography_msg.data = homography.reshape(-1).astype(float).tolist()
        self.homography_pub.publish(homography_msg)

        self._publish_debug(
            bev_bgr, map_points, bev_points, scores, inlier_mask,
            center_uv, source_msg)

        elapsed = time.perf_counter() - start
        self.get_logger().info(
            f'pose=({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}deg) '
            f'matches={len(map_points)} inliers={inlier_count}'
            f'({inlier_ratio:.2f}) time={elapsed:.3f}s')

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def _bev_metric_to_pixel(self, x, y, width, height):
        u = (self.bev_y_max - y) / (self.bev_y_max - self.bev_y_min) * (width - 1)
        v = (self.bev_x_max - x) / (self.bev_x_max - self.bev_x_min) * (height - 1)
        return [float(u), float(v)]

    def _map_pixel_to_world(self, point):
        width = self._map_bgr.shape[1]
        height = self._map_bgr.shape[0]
        u_norm = float(point[0]) / max(1, width - 1)
        v_norm = float(point[1]) / max(1, height - 1)
        x_norm, y_norm = (v_norm, u_norm) if self.map_swap_xy else (u_norm, v_norm)
        if self.map_flip_x:
            x_norm = 1.0 - x_norm
        if self.map_flip_y:
            y_norm = 1.0 - y_norm
        x = self.map_x_min + x_norm * (self.map_x_max - self.map_x_min)
        y = self.map_y_min + y_norm * (self.map_y_max - self.map_y_min)
        return x, y

    def _inside_map(self, point):
        return (
            np.isfinite(point).all()
            and 0.0 <= point[0] < self._map_bgr.shape[1]
            and 0.0 <= point[1] < self._map_bgr.shape[0]
        )

    # ------------------------------------------------------------------
    # ROS message helpers
    # ------------------------------------------------------------------

    def _make_pose(self, source_msg, x, y, yaw, inlier_ratio):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = source_msg.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        pos_var = max(0.01, (1.0 - inlier_ratio) ** 2)
        yaw_var = max(0.01, (1.0 - inlier_ratio) * 0.5)
        msg.pose.covariance[0] = pos_var
        msg.pose.covariance[7] = pos_var
        msg.pose.covariance[14] = 1.0
        msg.pose.covariance[21] = 1.0
        msg.pose.covariance[28] = 1.0
        msg.pose.covariance[35] = yaw_var
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

    def _publish_debug(
            self, bev_bgr, map_points, bev_points, scores,
            inlier_mask, center_uv, source_msg):
        map_view = self._map_bgr.copy()
        if center_uv is not None:
            cv2.drawMarker(
                map_view,
                tuple(np.round(center_uv).astype(int)),
                (0, 0, 255), cv2.MARKER_CROSS, 36, 4)

        target_h = max(240, self.debug_height)
        map_scale = target_h / map_view.shape[0]
        bev_scale = target_h / bev_bgr.shape[0]
        map_r = cv2.resize(map_view, None, fx=map_scale, fy=map_scale,
                           interpolation=cv2.INTER_AREA)
        bev_r = cv2.resize(bev_bgr, None, fx=bev_scale, fy=bev_scale,
                           interpolation=cv2.INTER_AREA)
        canvas = np.zeros((target_h, map_r.shape[1] + bev_r.shape[1], 3),
                          dtype=np.uint8)
        canvas[:, :map_r.shape[1]] = map_r
        canvas[:, map_r.shape[1]:] = bev_r

        indices = np.flatnonzero(inlier_mask)
        if len(indices) > self.max_debug_matches:
            ranked = indices[np.argsort(scores[indices])]
            indices = ranked[-self.max_debug_matches:]
        offset = map_r.shape[1]
        for idx in indices:
            color = tuple(int(c) for c in MATCH_COLORS_BGR[idx % len(MATCH_COLORS_BGR)])
            p0 = tuple(np.round(map_points[idx] * map_scale).astype(int))
            p1_local = np.round(bev_points[idx] * bev_scale).astype(int)
            p1 = (int(p1_local[0] + offset), int(p1_local[1]))
            cv2.line(canvas, p0, p1, color, 1, cv2.LINE_AA)
            cv2.circle(canvas, p0, 2, color, -1)
            cv2.circle(canvas, p1, 2, color, -1)

        self.debug_pub.publish(self._bgr_to_ros_image(canvas, source_msg))

    # ------------------------------------------------------------------
    # Image conversion
    # ------------------------------------------------------------------

    @staticmethod
    def _ros_image_to_bgr(msg):
        encoding = msg.encoding.lower()
        channel_map = {
            'mono8': 1, '8uc1': 1,
            'bgr8': 3, 'rgb8': 3,
            'bgra8': 4, 'rgba8': 4,
        }
        if encoding not in channel_map:
            raise ValueError(f'Unsupported image encoding: {msg.encoding}')
        ch = channel_map[encoding]
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = (rows[:, :msg.width * ch].reshape(msg.height, msg.width, ch).copy()
               if ch > 1
               else rows[:, :msg.width].reshape(msg.height, msg.width).copy())
        if encoding in ('mono8', '8uc1'):
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if encoding == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img

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
