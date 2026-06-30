#!/usr/bin/env python3
"""Generate visual assets for lecture05 vision navigation slides."""

from __future__ import annotations

import math
import random
import textwrap
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = ROOT / "lecture" / "assets" / "lecture05"
CAPTURE_DIR = ASSET_DIR / "captures"
DIAGRAM_DIR = ASSET_DIR / "diagrams"
TRACK_PATH = (
    ROOT
    / "system"
    / "simple_gazebo_track"
    / "models"
    / "simple_track"
    / "materials"
    / "textures"
    / "track.png"
)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    names = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    ]
    for name in names:
        path = Path(name)
        if path.exists():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def ensure_dirs() -> None:
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    DIAGRAM_DIR.mkdir(parents=True, exist_ok=True)


def fit_image(image: Image.Image, box: tuple[int, int]) -> Image.Image:
    image = image.convert("RGB")
    image.thumbnail(box, Image.Resampling.LANCZOS)
    return image


def draw_label(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    fill: str = "#101820",
    bg: str = "#ffffff",
    size: int = 26,
) -> None:
    label_font = font(size, bold=True)
    pad_x, pad_y = 12, 7
    bbox = draw.textbbox(xy, text, font=label_font)
    rect = (
        bbox[0] - pad_x,
        bbox[1] - pad_y,
        bbox[2] + pad_x,
        bbox[3] + pad_y,
    )
    draw.rounded_rectangle(rect, radius=8, fill=bg, outline="#d0d7de", width=2)
    draw.text(xy, text, font=label_font, fill=fill)


def draw_terminal_capture(
    title: str,
    command: str,
    body: str,
    output: Path,
    width: int = 1400,
    font_size: int = 26,
) -> None:
    mono = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        size=font_size,
    )
    title_font = font(font_size, bold=True)
    line_height = int(font_size * 1.45)
    margin = 28
    max_chars = max(40, (width - margin * 2) // max(8, int(mono.getlength("M"))))
    lines: list[tuple[str, str]] = []
    for idx, line in enumerate((f"$ {command}\n{body}").splitlines()):
        color = "#7aa2f7" if idx == 0 else "#e6edf3"
        wrapped = textwrap.wrap(
            line,
            width=max_chars,
            replace_whitespace=False,
            drop_whitespace=False,
            break_long_words=True,
        ) or [""]
        lines.extend((part, color) for part in wrapped)
    height = 52 + margin * 2 + line_height * len(lines)
    image = Image.new("RGB", (width, height), "#0d1117")
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, width, 52), fill="#30363d")
    draw.ellipse((18, 18, 32, 32), fill="#ff5f56")
    draw.ellipse((42, 18, 56, 32), fill="#ffbd2e")
    draw.ellipse((66, 18, 80, 32), fill="#27c93f")
    draw.text((100, 12), title, font=title_font, fill="#f0f6fc")
    y = 52 + margin
    for line, color in lines:
        draw.text((margin, y), line, font=mono, fill=color)
        y += line_height
    image.save(output)


def generate_track_overview() -> None:
    track = Image.open(TRACK_PATH).convert("RGB")
    canvas = Image.new("RGB", (1600, 900), "#f6f8fa")
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 1600, 74), fill="#1f2937")
    draw.text(
        (34, 20),
        "AAC Gazebo simple_track map texture",
        font=font(32, bold=True),
        fill="#ffffff",
    )
    resized = fit_image(track, (1440, 760))
    x = (1600 - resized.width) // 2
    y = 100
    canvas.paste(resized, (x, y))
    draw.rectangle((x, y, x + resized.width, y + resized.height), outline="#111827", width=3)

    sx = x + int(resized.width * 0.035)
    sy = y + int(resized.height * 0.49)
    draw_label(draw, (sx - 26, sy - 62), "spawn / start", bg="#fff7ed", fill="#9a3412", size=24)
    draw.ellipse((sx - 14, sy - 14, sx + 14, sy + 14), fill="#ef4444", outline="#7f1d1d", width=3)
    draw.line((sx, sy, sx + 120, sy), fill="#ef4444", width=5)

    axis_x0, axis_y0 = 1260, 780
    draw.line((axis_x0, axis_y0, axis_x0 + 170, axis_y0), fill="#2563eb", width=6)
    draw.polygon(
        [
            (axis_x0 + 190, axis_y0),
            (axis_x0 + 160, axis_y0 - 16),
            (axis_x0 + 160, axis_y0 + 16),
        ],
        fill="#2563eb",
    )
    draw.text((axis_x0 + 205, axis_y0 - 20), "world +Y", font=font(24, True), fill="#2563eb")
    draw.line((axis_x0, axis_y0, axis_x0, axis_y0 - 150), fill="#16a34a", width=6)
    draw.polygon(
        [
            (axis_x0, axis_y0 - 170),
            (axis_x0 - 16, axis_y0 - 140),
            (axis_x0 + 16, axis_y0 - 140),
        ],
        fill="#16a34a",
    )
    draw.text((axis_x0 + 20, axis_y0 - 180), "world +X", font=font(24, True), fill="#16a34a")
    draw_label(draw, (50, 820), "map bounds: X [-6, 6] m, Y [-8, 8] m", bg="#eff6ff", fill="#1d4ed8")
    canvas.save(CAPTURE_DIR / "simulator_track_overview.png")


def generate_match_debug() -> None:
    track = Image.open(TRACK_PATH).convert("RGB")
    left = fit_image(track.copy(), (720, 680))
    crop = track.crop((760, 730, 2460, 1830)).rotate(-7, expand=True, fillcolor=(70, 70, 70))
    right = fit_image(crop, (720, 680))

    canvas = Image.new("RGB", (1600, 900), "#101820")
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 1600, 72), fill="#0b1220")
    draw.text((36, 20), "Visual localization debug: map <-> BEV inlier matches", font=font(30, True), fill="#ffffff")

    lx, ly = 60, 116
    rx, ry = 820, 116
    draw.rounded_rectangle((lx - 16, ly - 16, lx + left.width + 16, ly + left.height + 16), radius=14, fill="#ffffff")
    draw.rounded_rectangle((rx - 16, ry - 16, rx + right.width + 16, ry + right.height + 16), radius=14, fill="#ffffff")
    canvas.paste(left, (lx, ly))
    canvas.paste(right, (rx, ry))
    draw_label(draw, (lx + 18, ly + 18), "global map track.png", bg="#ffffff", fill="#1f2937", size=24)
    draw_label(draw, (rx + 18, ry + 18), "vehicle BEV /ipm_bev/fused", bg="#ffffff", fill="#1f2937", size=24)

    random.seed(5)
    colors = ["#22c55e", "#06b6d4", "#e879f9", "#facc15", "#fb7185"]
    for i in range(42):
        px = lx + random.randint(90, left.width - 80)
        py = ly + random.randint(120, left.height - 70)
        qx = rx + int((px - lx) / left.width * right.width) + random.randint(-70, 70)
        qy = ry + int((py - ly) / left.height * right.height) + random.randint(-55, 55)
        qx = max(rx + 20, min(rx + right.width - 20, qx))
        qy = max(ry + 20, min(ry + right.height - 20, qy))
        color = colors[i % len(colors)]
        draw.line((px, py, qx, qy), fill=color, width=2)
        draw.ellipse((px - 4, py - 4, px + 4, py + 4), fill=color)
        draw.ellipse((qx - 4, qy - 4, qx + 4, qy + 4), fill=color)

    center = (rx + right.width // 2, ry + right.height // 2)
    draw.line((center[0] - 28, center[1], center[0] + 28, center[1]), fill="#ef4444", width=5)
    draw.line((center[0], center[1] - 28, center[0], center[1] + 28), fill="#ef4444", width=5)
    draw_label(draw, (60, 820), "publish: /visual_localization/debug_matches", bg="#ecfeff", fill="#0e7490", size=24)
    draw_label(draw, (840, 820), "RANSAC keeps geometry-consistent matches", bg="#f0fdf4", fill="#166534", size=22)
    canvas.save(CAPTURE_DIR / "simulator_matching_debug.png")


def generate_feature_concept() -> None:
    track = Image.open(TRACK_PATH).convert("RGB")
    crop = track.crop((200, 220, 1340, 940))
    crop = fit_image(crop, (900, 560))
    canvas = Image.new("RGB", (1400, 780), "#f8fafc")
    draw = ImageDraw.Draw(canvas)
    canvas.paste(crop, (60, 110))
    draw.rectangle((60, 110, 60 + crop.width, 110 + crop.height), outline="#111827", width=3)
    random.seed(17)
    for i in range(34):
        x = 60 + random.randint(90, crop.width - 90)
        y = 110 + random.randint(90, crop.height - 90)
        r = random.randint(10, 25)
        angle = random.random() * math.tau
        draw.ellipse((x - r, y - r, x + r, y + r), outline="#22c55e", width=3)
        draw.line((x, y, x + math.cos(angle) * r, y + math.sin(angle) * r), fill="#22c55e", width=3)
    draw.text((58, 34), "Classical local features", font=font(34, True), fill="#0f172a")
    draw.text((1000, 130), "SIFT", font=font(36, True), fill="#1d4ed8")
    draw.text((1000, 180), "scale-space keypoint", font=font(22), fill="#334155")
    draw.text((1000, 212), "orientation histogram", font=font(22), fill="#334155")
    draw.text((1000, 244), "128-D float descriptor", font=font(22), fill="#334155")
    draw.text((1000, 345), "ORB", font=font(36, True), fill="#b45309")
    draw.text((1000, 395), "FAST corners + orientation", font=font(22), fill="#334155")
    draw.text((1000, 427), "rotated BRIEF pattern", font=font(22), fill="#334155")
    draw.text((1000, 459), "binary descriptor", font=font(22), fill="#334155")
    for row in range(5):
        for col in range(16):
            x0 = 1000 + col * 18
            y0 = 540 + row * 22
            fill = "#111827" if (row * 17 + col * 7) % 5 in (0, 2) else "#e2e8f0"
            draw.rectangle((x0, y0, x0 + 13, y0 + 13), fill=fill)
    draw_label(draw, (84, 686), "keypoint = location + scale + orientation + descriptor", bg="#ffffff", fill="#0f172a", size=24)
    canvas.save(CAPTURE_DIR / "feature_keypoints_on_track.png")


def generate_svg_assets() -> None:
    svg_common = """<defs>
  <style>
    text { font-family: DejaVu Sans, Arial, sans-serif; fill:#111827; }
    .title { font-size:34px; font-weight:700; }
    .label { font-size:23px; font-weight:700; }
    .small { font-size:18px; }
    .box { fill:#ffffff; stroke:#cbd5e1; stroke-width:3; rx:14; }
    .blue { fill:#eff6ff; stroke:#60a5fa; }
    .green { fill:#ecfdf5; stroke:#34d399; }
    .orange { fill:#fff7ed; stroke:#fb923c; }
    .purple { fill:#faf5ff; stroke:#c084fc; }
    .dark { fill:#1f2937; }
    .arrow { stroke:#334155; stroke-width:4; fill:none; marker-end:url(#arrow); }
  </style>
  <marker id="arrow" markerWidth="14" markerHeight="14" refX="10" refY="4" orient="auto" markerUnits="strokeWidth">
    <path d="M0,0 L10,4 L0,8 z" fill="#334155" />
  </marker>
</defs>"""

    (DIAGRAM_DIR / "ros_image_topic_flow.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">ROS image topic flow in AAC2026</text>
<rect x="60" y="130" width="230" height="120" class="box blue"/>
<text x="94" y="178" class="label">Gazebo cameras</text>
<text x="94" y="213" class="small">/camera_front/image_raw</text>
<text x="94" y="240" class="small">/camera_rear/image_raw</text>
<path d="M300 190 H390" class="arrow"/>
<rect x="405" y="130" width="230" height="120" class="box orange"/>
<text x="455" y="178" class="label">IPM BEV node</text>
<text x="452" y="213" class="small">perspective warp</text>
<text x="452" y="240" class="small">front + rear fusion</text>
<path d="M645 190 H735" class="arrow"/>
<rect x="750" y="130" width="230" height="120" class="box green"/>
<text x="795" y="178" class="label">/ipm_bev/fused</text>
<text x="790" y="213" class="small">sensor_msgs/Image</text>
<text x="790" y="240" class="small">bgr8, 480 x 800</text>
<path d="M990 190 H1080" class="arrow"/>
<rect x="1090" y="130" width="150" height="120" class="box purple"/>
<text x="1126" y="178" class="label">Matcher</text>
<text x="1120" y="213" class="small">OpenCV</text>
<text x="1120" y="240" class="small">Deep</text>
<rect x="140" y="390" width="260" height="120" class="box"/>
<text x="190" y="438" class="label">Image checks</text>
<text x="178" y="473" class="small">rqt_image_view</text>
<text x="178" y="500" class="small">ros2 topic hz</text>
<path d="M865 260 C865 330 380 310 300 390" class="arrow"/>
<rect x="520" y="390" width="260" height="120" class="box"/>
<text x="580" y="438" class="label">Debug image</text>
<text x="560" y="473" class="small">/visual_localization</text>
<text x="560" y="500" class="small">/debug_matches</text>
<path d="M1165 260 C1165 360 820 340 705 390" class="arrow"/>
<rect x="900" y="390" width="260" height="120" class="box"/>
<text x="970" y="438" class="label">Pose output</text>
<text x="940" y="473" class="small">PoseWithCovariance</text>
<text x="940" y="500" class="small">homography, counts</text>
<path d="M1165 260 C1165 365 1110 350 1060 390" class="arrow"/>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "visual_localization_pipeline.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">BEV-to-map visual localization pipeline</text>
<rect x="60" y="130" width="190" height="110" class="box blue"/>
<text x="96" y="180" class="label">Map image</text>
<text x="90" y="214" class="small">track.png</text>
<rect x="60" y="330" width="190" height="110" class="box green"/>
<text x="96" y="380" class="label">BEV image</text>
<text x="92" y="414" class="small">/ipm_bev/fused</text>
<path d="M260 185 H365" class="arrow"/>
<path d="M260 385 H365" class="arrow"/>
<rect x="380" y="200" width="220" height="170" class="box orange"/>
<text x="428" y="250" class="label">Matcher</text>
<text x="420" y="285" class="small">SIFT / ORB / AKAZE</text>
<text x="420" y="315" class="small">EfficientLoFTR</text>
<text x="420" y="345" class="small">MINIMA</text>
<path d="M610 285 H720" class="arrow"/>
<rect x="735" y="200" width="190" height="170" class="box purple"/>
<text x="792" y="250" class="label">RANSAC</text>
<text x="775" y="285" class="small">homography H</text>
<text x="775" y="315" class="small">inlier mask</text>
<text x="775" y="345" class="small">reject outliers</text>
<path d="M935 285 H1035" class="arrow"/>
<rect x="1050" y="200" width="190" height="170" class="box green"/>
<text x="1105" y="250" class="label">Pose</text>
<text x="1086" y="285" class="small">center pixel -> x,y</text>
<text x="1086" y="315" class="small">forward pixel -> yaw</text>
<text x="1086" y="345" class="small">publish TF/pose</text>
<rect x="280" y="515" width="740" height="90" class="box"/>
<text x="322" y="552" class="label">Core idea</text>
<text x="322" y="585" class="small">If the BEV road markings align with the global map, vehicle pose is the projected BEV center.</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "webui_to_ros_integration.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">From Image Matching WebUI to AAC ROS integration</text>
<rect x="70" y="130" width="340" height="430" class="box blue"/>
<text x="115" y="180" class="label">Image Matching WebUI</text>
<rect x="105" y="220" width="125" height="95" fill="#e5e7eb" stroke="#94a3b8" stroke-width="3"/>
<rect x="250" y="220" width="125" height="95" fill="#e5e7eb" stroke="#94a3b8" stroke-width="3"/>
<text x="120" y="348" class="small">upload image A/B</text>
<rect x="105" y="380" width="270" height="48" fill="#ffffff" stroke="#94a3b8" stroke-width="2" rx="8"/>
<text x="128" y="412" class="small">matcher: OmniGlue / ELoFTR / MINIMA</text>
<rect x="105" y="450" width="270" height="60" fill="#111827" rx="8"/>
<text x="140" y="488" class="small" fill="#ffffff">visualize correspondences</text>
<path d="M430 345 H550" class="arrow"/>
<rect x="570" y="130" width="270" height="430" class="box orange"/>
<text x="615" y="180" class="label">Integration choice</text>
<text x="615" y="232" class="small">1. select backend</text>
<text x="615" y="270" class="small">2. define input format</text>
<text x="615" y="308" class="small">3. add checkpoint/cache</text>
<text x="615" y="346" class="small">4. convert matches to H</text>
<text x="615" y="384" class="small">5. publish ROS topics</text>
<path d="M860 345 H980" class="arrow"/>
<rect x="1000" y="130" width="220" height="430" class="box green"/>
<text x="1048" y="180" class="label">AAC ROS node</text>
<text x="1040" y="232" class="small">subscribe Image</text>
<text x="1040" y="270" class="small">run matcher</text>
<text x="1040" y="308" class="small">findHomography</text>
<text x="1040" y="346" class="small">publish pose</text>
<text x="1040" y="384" class="small">publish debug</text>
<text x="1040" y="422" class="small">optional TF</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "matcher_backend_landscape.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">Matcher backend landscape for this lecture</text>
<rect x="70" y="135" width="300" height="380" class="box blue"/>
<text x="116" y="190" class="label">Classical sparse</text>
<text x="110" y="245" class="small">SIFT: robust float descriptors</text>
<text x="110" y="285" class="small">ORB: fast binary descriptors</text>
<text x="110" y="325" class="small">AKAZE: nonlinear scale space</text>
<text x="110" y="390" class="small">Good for code tracing and CPU tests</text>
<rect x="490" y="135" width="300" height="380" class="box orange"/>
<text x="545" y="190" class="label">Detector-free deep</text>
<text x="530" y="245" class="small">LoFTR family: coarse-to-fine</text>
<text x="530" y="285" class="small">EfficientLoFTR: faster semi-dense</text>
<text x="530" y="325" class="small">Works in low texture areas</text>
<text x="530" y="390" class="small">Needs GPU/cache planning</text>
<rect x="910" y="135" width="300" height="380" class="box purple"/>
<text x="964" y="190" class="label">Generalizable / modal</text>
<text x="950" y="245" class="small">OmniGlue: foundation guidance</text>
<text x="950" y="285" class="small">MINIMA: cross-modal training</text>
<text x="950" y="325" class="small">RoMa / SP+LightGlue options</text>
<text x="950" y="390" class="small">Best explored in WebUI first</text>
<path d="M370 330 H490" class="arrow"/>
<path d="M790 330 H910" class="arrow"/>
<text x="180" y="590" class="small">Debug first with OpenCV, then compare deep matchers under the same ROS topic contract.</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "homography_pose_projection.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">Pose from homography: project BEV center and forward probe</text>
<rect x="110" y="145" width="420" height="420" fill="#eef2f7" stroke="#94a3b8" stroke-width="4"/>
<text x="225" y="120" class="label">BEV image</text>
<line x1="320" y1="530" x2="320" y2="190" stroke="#64748b" stroke-width="3" stroke-dasharray="12 10"/>
<line x1="145" y1="355" x2="495" y2="355" stroke="#64748b" stroke-width="3" stroke-dasharray="12 10"/>
<circle cx="320" cy="355" r="13" fill="#ef4444"/>
<circle cx="320" cy="250" r="13" fill="#22c55e"/>
<text x="340" y="365" class="small">center</text>
<text x="340" y="258" class="small">forward probe</text>
<path d="M550 355 H720" class="arrow"/>
<text x="590" y="330" class="label">H</text>
<rect x="750" y="145" width="420" height="420" fill="#f0fdf4" stroke="#86efac" stroke-width="4"/>
<text x="870" y="120" class="label">Map image</text>
<path d="M890 470 C925 380 1010 340 1055 235" stroke="#475569" stroke-width="10" fill="none"/>
<circle cx="925" cy="380" r="13" fill="#ef4444"/>
<circle cx="975" cy="310" r="13" fill="#22c55e"/>
<line x1="925" y1="380" x2="975" y2="310" stroke="#ef4444" stroke-width="5"/>
<text x="945" y="392" class="small">x, y</text>
<text x="995" y="320" class="small">yaw</text>
<rect x="225" y="612" width="850" height="58" class="box"/>
<text x="270" y="650" class="small">center_uv and forward_uv are converted to world coordinates with map_swap_xy / map_flip_x / map_flip_y.</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "global_local_coordinate_problem.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">Lecture goal: estimate local vehicle pose in global map coordinates</text>
<rect x="62" y="112" width="790" height="500" fill="#f0fdf4" stroke="#86efac" stroke-width="4" rx="16"/>
<text x="92" y="158" class="label">Global / world frame: map</text>
<text x="92" y="192" class="small">fixed to the track texture and never moves</text>

<path d="M185 395 C240 240 410 205 560 286 C675 350 682 478 540 540 C390 608 220 545 178 455 C160 420 165 405 185 395"
      stroke="#475569" stroke-width="34" fill="none" stroke-linecap="round"/>
<path d="M185 395 C240 240 410 205 560 286 C675 350 682 478 540 540 C390 608 220 545 178 455 C160 420 165 405 185 395"
      stroke="#f8fafc" stroke-width="4" fill="none" stroke-dasharray="28 30"/>

<line x1="142" y1="560" x2="390" y2="560" stroke="#2563eb" stroke-width="6"/>
<polygon points="414,560 378,541 378,579" fill="#2563eb"/>
<text x="425" y="570" class="small">+Y_map</text>
<line x1="142" y1="560" x2="142" y2="295" stroke="#16a34a" stroke-width="6"/>
<polygon points="142,270 123,306 161,306" fill="#16a34a"/>
<text x="165" y="282" class="small">+X_map</text>
<circle cx="142" cy="560" r="8" fill="#111827"/>
<text x="92" y="590" class="small">map origin</text>

<line x1="142" y1="338" x2="510" y2="338" stroke="#94a3b8" stroke-width="3" stroke-dasharray="9 9"/>
<line x1="510" y1="560" x2="510" y2="338" stroke="#94a3b8" stroke-width="3" stroke-dasharray="9 9"/>
<text x="300" y="326" class="small">y_map</text>
<text x="523" y="455" class="small">x_map</text>

<g transform="translate(510 338) rotate(-25)">
  <rect x="-48" y="-23" width="96" height="46" rx="9" fill="#ef4444" stroke="#991b1b" stroke-width="3"/>
  <polygon points="48,0 72,-15 72,15" fill="#fecaca" stroke="#991b1b" stroke-width="2"/>
  <circle cx="0" cy="0" r="7" fill="#111827"/>
  <line x1="0" y1="0" x2="150" y2="0" stroke="#16a34a" stroke-width="6"/>
  <polygon points="174,0 138,-19 138,19" fill="#16a34a"/>
  <line x1="0" y1="0" x2="0" y2="-120" stroke="#2563eb" stroke-width="6"/>
  <polygon points="0,-145 -19,-109 19,-109" fill="#2563eb"/>
  <path d="M70 -64 A92 92 0 0 1 122 -8" stroke="#f97316" stroke-width="5" fill="none"/>
</g>
<text x="660" y="286" class="small">+X_base forward</text>
<text x="440" y="195" class="small">+Y_base left</text>
<text x="610" y="310" class="small" fill="#c2410c">yaw_map</text>
<text x="532" y="365" class="small">base_footprint</text>

<rect x="895" y="145" width="315" height="330" class="box"/>
<text x="935" y="195" class="label">What changes?</text>
<text x="935" y="242" class="small">map frame: fixed</text>
<text x="935" y="280" class="small">base frame: moves with vehicle</text>
<text x="935" y="318" class="small">camera/BEV: local observation</text>
<text x="935" y="382" class="label">Unknown</text>
<text x="935" y="425" class="small">T_map_base = [x, y, yaw]</text>

<rect x="165" y="635" width="950" height="58" class="box orange"/>
<text x="205" y="672" class="small">Visual localization estimates where the moving local frame sits inside the fixed global map.</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "ipm_projection_math.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">IPM math: ground grid to camera pixels to BEV image</text>
<rect x="65" y="125" width="310" height="420" class="box green"/>
<text x="105" y="175" class="label">BEV ground grid</text>
<text x="105" y="220" class="small">for each row, col:</text>
<text x="105" y="258" class="small">x = x_max - (row+0.5) res</text>
<text x="105" y="292" class="small">y = y_max - (col+0.5) res</text>
<text x="105" y="326" class="small">P_base = [x, y, ground_z]</text>
<line x1="115" y1="405" x2="315" y2="405" stroke="#2563eb" stroke-width="5"/>
<line x1="115" y1="405" x2="115" y2="305" stroke="#16a34a" stroke-width="5"/>
<path d="M385 335 H510" class="arrow"/>
<rect x="525" y="125" width="300" height="420" class="box blue"/>
<text x="565" y="175" class="label">TF transform</text>
<text x="565" y="220" class="small">T_base_camera from TF</text>
<text x="565" y="258" class="small">P_camera = inv(T) P_base</text>
<text x="565" y="310" class="small">link_x_forward:</text>
<text x="565" y="344" class="small">P_opt = [-Yc, -Zc, Xc]</text>
<text x="565" y="386" class="small">keep only depth > min</text>
<path d="M835 335 H960" class="arrow"/>
<rect x="975" y="125" width="245" height="420" class="box orange"/>
<text x="1018" y="175" class="label">Projection</text>
<text x="1010" y="220" class="small">p ~ K [I|0] P_opt</text>
<text x="1010" y="258" class="small">u = fx X/Z + cx</text>
<text x="1010" y="292" class="small">v = fy Y/Z + cy</text>
<text x="1010" y="344" class="small">map_x(row,col)=u</text>
<text x="1010" y="378" class="small">map_y(row,col)=v</text>
<text x="1010" y="430" class="small">cv::remap(image, BEV)</text>
<rect x="165" y="600" width="950" height="62" class="box"/>
<text x="200" y="640" class="small">Implementation builds a lookup table from BEV pixels to camera pixels, then samples the camera image.</text>
</svg>
""",
        encoding="utf-8",
    )


def generate_terminal_assets() -> None:
    draw_terminal_capture(
        "ROS image topics to inspect",
        'ros2 topic list | grep -E "camera|ipm|visual"',
        """/camera_front/image_raw
/camera_rear/image_raw
/camera_info
/ipm_bev/front
/ipm_bev/rear
/ipm_bev/fused
/ipm_bev/debug
/visual_localization/pose
/visual_localization/debug_matches""",
        CAPTURE_DIR / "terminal_ros_image_topics.png",
    )
    draw_terminal_capture(
        "Visual localization health check",
        "ros2 topic hz /ipm_bev/fused && ros2 topic echo /visual_localization/match_count --once",
        """average rate: 10.21
min: 0.081s max: 0.129s std dev: 0.012s window: 20
data: 238""",
        CAPTURE_DIR / "terminal_visual_localization_check.png",
    )


def main() -> None:
    ensure_dirs()
    generate_track_overview()
    generate_match_debug()
    generate_feature_concept()
    generate_svg_assets()
    generate_terminal_assets()


if __name__ == "__main__":
    main()
