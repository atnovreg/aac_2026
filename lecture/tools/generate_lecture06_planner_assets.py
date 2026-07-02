#!/usr/bin/env python3
"""Generate visual assets for lecture06 global path planning slides."""

from __future__ import annotations

import math
import random
import shutil
import textwrap
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = ROOT / "lecture" / "assets" / "lecture06"
CAPTURE_DIR = ASSET_DIR / "captures"
DIAGRAM_DIR = ASSET_DIR / "diagrams"
TRACK_PREVIEW = ROOT / "planning" / "track_line_export" / "track_texture_lines_preview.png"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    ]
    for candidate in candidates:
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size=size)
    return ImageFont.load_default()


def ensure_dirs() -> None:
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    DIAGRAM_DIR.mkdir(parents=True, exist_ok=True)


def draw_terminal_capture(
    title: str,
    command: str,
    body: str,
    output: Path,
    width: int = 1420,
    font_size: int = 25,
) -> None:
    mono = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        size=font_size,
    )
    title_font = font(font_size, bold=True)
    line_height = int(font_size * 1.45)
    margin = 28
    max_chars = max(44, (width - margin * 2) // max(8, int(mono.getlength("M"))))
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


def copy_track_preview() -> None:
    if TRACK_PREVIEW.exists():
        shutil.copy2(TRACK_PREVIEW, CAPTURE_DIR / "track_texture_lines_preview.png")


def generate_local_segment_capture() -> None:
    if not TRACK_PREVIEW.exists():
        return
    image = Image.open(TRACK_PREVIEW).convert("RGB")
    image.thumbnail((1500, 850), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", (1600, 900), "#f8fafc")
    x = (1600 - image.width) // 2
    y = 72
    canvas.paste(image, (x, y))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 1600, 64), fill="#1f2937")
    draw.text((34, 18), "Global centerline and local lookahead segment", font=font(30, True), fill="#ffffff")
    # Approximate vehicle and local segment positions on the scaled preview.
    cx = x + int(image.width * 0.36)
    cy = y + int(image.height * 0.74)
    points = [
        (cx, cy),
        (cx + 110, cy + 5),
        (cx + 230, cy - 8),
        (cx + 360, cy - 25),
        (cx + 500, cy - 38),
    ]
    for a, b in zip(points, points[1:]):
        draw.line((a, b), fill="#00e5ff", width=10)
    for px, py in points:
        draw.ellipse((px - 10, py - 10, px + 10, py + 10), fill="#00e5ff", outline="#0e7490", width=3)
    draw.polygon(
        [
            (cx - 28, cy - 16),
            (cx + 38, cy),
            (cx - 28, cy + 16),
        ],
        fill="#ef4444",
        outline="#7f1d1d",
    )
    label_font = font(26, True)
    draw.rounded_rectangle((48, 800, 705, 858), radius=10, fill="#ffffff", outline="#cbd5e1", width=2)
    draw.text((70, 816), "/path/center in map frame -> /local_path/global lookahead", font=label_font, fill="#0f172a")
    canvas.save(CAPTURE_DIR / "track_local_segment_capture.png")


def generate_terminal_assets() -> None:
    draw_terminal_capture(
        "Planner topics",
        'ros2 topic list | grep -E "path|pure_pursuit|ackermann_auto"',
        """/path/left
/path/center
/path/right
/local_path/global
/local_path/body
/pure_pursuit/target_point
/pure_pursuit/curvature_markers
/ackermann_auto""",
        CAPTURE_DIR / "terminal_planner_topics.png",
    )
    draw_terminal_capture(
        "Planner quick checks",
        "ros2 topic echo --once /local_path/body && ros2 topic echo --once /ackermann_auto",
        """header:
  frame_id: base_footprint
poses: [ ... lookahead path points ... ]
---
drive:
  speed: 0.74
  steering_angle: -0.18""",
        CAPTURE_DIR / "terminal_planner_check.png",
    )


def generate_svg_assets() -> None:
    svg_common = """<defs>
  <style>
    text { font-family: DejaVu Sans, Arial, sans-serif; fill:#111827; }
    .title { font-size:34px; font-weight:700; }
    .label { font-size:23px; font-weight:700; }
    .small { font-size:18px; }
    .tiny { font-size:15px; }
    .box { fill:#ffffff; stroke:#cbd5e1; stroke-width:3; rx:14; }
    .blue { fill:#eff6ff; stroke:#60a5fa; }
    .green { fill:#ecfdf5; stroke:#34d399; }
    .orange { fill:#fff7ed; stroke:#fb923c; }
    .purple { fill:#faf5ff; stroke:#c084fc; }
    .red { fill:#fef2f2; stroke:#f87171; }
    .arrow { stroke:#334155; stroke-width:4; fill:none; marker-end:url(#arrow); }
  </style>
  <marker id="arrow" markerWidth="14" markerHeight="14" refX="10" refY="4" orient="auto" markerUnits="strokeWidth">
    <path d="M0,0 L10,4 L0,8 z" fill="#334155" />
  </marker>
</defs>"""

    (DIAGRAM_DIR / "global_planning_problem.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">Global planning problem in map frame</text>
<rect x="70" y="115" width="760" height="500" class="box green"/>
<text x="105" y="165" class="label">map frame</text>
<line x1="135" y1="550" x2="355" y2="550" stroke="#2563eb" stroke-width="6"/>
<polygon points="378,550 343,532 343,568" fill="#2563eb"/>
<text x="390" y="560" class="small">+Y_map</text>
<line x1="135" y1="550" x2="135" y2="310" stroke="#16a34a" stroke-width="6"/>
<polygon points="135,286 117,321 153,321" fill="#16a34a"/>
<text x="158" y="300" class="small">+X_map</text>
<path d="M205 430 C245 240 445 220 610 310 C725 375 720 505 565 555 C410 610 230 555 205 430"
  stroke="#475569" stroke-width="32" fill="none" stroke-linecap="round"/>
<path d="M205 430 C245 240 445 220 610 310 C725 375 720 505 565 555 C410 610 230 555 205 430"
  stroke="#fde047" stroke-width="8" fill="none" stroke-linecap="round"/>
<circle cx="276" cy="475" r="18" fill="#ef4444"/>
<text x="235" y="515" class="small">start</text>
<circle cx="650" cy="318" r="18" fill="#22c55e"/>
<text x="615" y="290" class="small">goal</text>
<path d="M276 475 C335 555 520 545 650 318" stroke="#06b6d4" stroke-width="9" fill="none"/>
<rect x="880" y="150" width="325" height="385" class="box"/>
<text x="920" y="205" class="label">Given</text>
<text x="920" y="248" class="small">ego pose in map</text>
<text x="920" y="286" class="small">map / road graph</text>
<text x="920" y="324" class="small">goal or mission</text>
<text x="920" y="386" class="label">Find</text>
<text x="920" y="428" class="small">route / path in map</text>
<text x="920" y="466" class="small">then local path</text>
<text x="920" y="504" class="small">then control target</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "road_graph_route_planning.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">Road graph route planning</text>
<rect x="70" y="110" width="1140" height="500" class="box"/>
<path d="M150 410 C240 290 380 285 485 380" stroke="#94a3b8" stroke-width="42" fill="none" stroke-linecap="round"/>
<path d="M480 380 C600 475 760 475 895 365" stroke="#94a3b8" stroke-width="42" fill="none" stroke-linecap="round"/>
<path d="M485 380 C575 255 720 235 880 260" stroke="#cbd5e1" stroke-width="38" fill="none" stroke-linecap="round"/>
<path d="M895 365 C970 305 1030 250 1120 200" stroke="#cbd5e1" stroke-width="38" fill="none" stroke-linecap="round"/>
<path d="M150 410 C240 290 380 285 485 380 C600 475 760 475 895 365" stroke="#facc15" stroke-width="10" fill="none" stroke-linecap="round"/>
<g fill="#ffffff" stroke="#1f2937" stroke-width="4">
  <circle cx="150" cy="410" r="20"/><circle cx="485" cy="380" r="20"/>
  <circle cx="895" cy="365" r="20"/><circle cx="880" cy="260" r="20"/>
  <circle cx="1120" cy="200" r="20"/>
</g>
<circle cx="150" cy="410" r="12" fill="#ef4444"/>
<circle cx="895" cy="365" r="12" fill="#22c55e"/>
<text x="112" y="455" class="small">start lanelet</text>
<text x="852" y="410" class="small">goal lanelet</text>
<text x="508" y="420" class="small">edge cost = length + turn + rule penalty</text>
<rect x="155" y="545" width="930" height="46" class="orange box"/>
<text x="185" y="575" class="small">Search over directed lane/road segments: Dijkstra, A*, time cost, rule-aware route.</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "planning_stack_layers.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">Planning stack: route, behavior, motion, control</text>
<rect x="90" y="125" width="230" height="135" class="box blue"/>
<text x="130" y="175" class="label">Mission / Route</text>
<text x="130" y="212" class="small">road graph</text>
<text x="130" y="240" class="small">start -> goal</text>
<path d="M330 192 H420" class="arrow"/>
<rect x="435" y="125" width="230" height="135" class="box green"/>
<text x="475" y="175" class="label">Behavior</text>
<text x="475" y="212" class="small">lane keep/change</text>
<text x="475" y="240" class="small">stop/yield</text>
<path d="M675 192 H765" class="arrow"/>
<rect x="780" y="125" width="230" height="135" class="box orange"/>
<text x="820" y="175" class="label">Motion</text>
<text x="820" y="212" class="small">smooth path</text>
<text x="820" y="240" class="small">velocity profile</text>
<path d="M1020 192 H1110" class="arrow"/>
<rect x="1120" y="125" width="110" height="135" class="box purple"/>
<text x="1140" y="175" class="label">Control</text>
<text x="1140" y="212" class="small">steer</text>
<text x="1140" y="240" class="small">speed</text>
<rect x="140" y="370" width="1000" height="100" class="box"/>
<text x="180" y="410" class="label">AAC2026 lecture scope</text>
<text x="180" y="445" class="small">Road graph concept first. Current simulator uses extracted waypoint centerline as a simplified global path.</text>
<rect x="195" y="535" width="220" height="80" class="box blue"/>
<text x="230" y="584" class="small">track_path_publisher</text>
<path d="M425 575 H505" class="arrow"/>
<rect x="520" y="535" width="230" height="80" class="box green"/>
<text x="552" y="584" class="small">local_path_planner</text>
<path d="M760 575 H840" class="arrow"/>
<rect x="855" y="535" width="210" height="80" class="box orange"/>
<text x="900" y="584" class="small">pure_pursuit</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "aac_planner_pipeline.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">AAC2026 waypoint planning pipeline</text>
<rect x="60" y="150" width="190" height="105" class="box blue"/>
<text x="96" y="192" class="label">CSV lines</text>
<text x="92" y="228" class="small">left / center / right</text>
<path d="M260 202 H345" class="arrow"/>
<rect x="360" y="150" width="230" height="105" class="box green"/>
<text x="390" y="192" class="label">track_path</text>
<text x="390" y="228" class="small">/path/center in map</text>
<path d="M600 202 H685" class="arrow"/>
<rect x="700" y="150" width="250" height="105" class="box orange"/>
<text x="735" y="192" class="label">local planner</text>
<text x="735" y="228" class="small">/local_path/body</text>
<path d="M960 202 H1045" class="arrow"/>
<rect x="1060" y="150" width="170" height="105" class="box purple"/>
<text x="1088" y="192" class="label">controller</text>
<text x="1088" y="228" class="small">/ackermann_auto</text>
<rect x="155" y="375" width="970" height="170" class="box"/>
<text x="205" y="420" class="label">Frame contract</text>
<text x="205" y="460" class="small">/path/center: nav_msgs/Path, frame_id = map</text>
<text x="205" y="492" class="small">/odom_gt or /odom_estimated: vehicle pose in the same map frame</text>
<text x="205" y="524" class="small">/local_path/body: same short path transformed into base_footprint</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "global_to_body_transform.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">From map-frame path to body-frame local path</text>
<rect x="80" y="120" width="520" height="420" class="box green"/>
<text x="120" y="165" class="label">map frame</text>
<path d="M150 430 C230 300 380 290 510 380" stroke="#facc15" stroke-width="9" fill="none"/>
<circle cx="285" cy="348" r="14" fill="#ef4444"/>
<text x="300" y="342" class="small">ego pose (rx, ry, ryaw)</text>
<path d="M285 348 L430 322" stroke="#06b6d4" stroke-width="8"/>
<path d="M610 330 H735" class="arrow"/>
<rect x="755" y="120" width="440" height="420" class="box blue"/>
<text x="800" y="165" class="label">body frame</text>
<circle cx="930" cy="390" r="14" fill="#ef4444"/>
<line x1="930" y1="390" x2="1110" y2="390" stroke="#16a34a" stroke-width="6"/>
<polygon points="1133,390 1098,372 1098,408" fill="#16a34a"/>
<text x="1060" y="420" class="small">+X forward</text>
<line x1="930" y1="390" x2="930" y2="230" stroke="#2563eb" stroke-width="6"/>
<polygon points="930,207 912,242 948,242" fill="#2563eb"/>
<text x="950" y="230" class="small">+Y left</text>
<path d="M930 390 C990 355 1045 330 1115 315" stroke="#06b6d4" stroke-width="8" fill="none"/>
<rect x="235" y="600" width="820" height="62" class="box"/>
<text x="270" y="640" class="small">body_x = cos(yaw) dx + sin(yaw) dy, body_y = -sin(yaw) dx + cos(yaw) dy</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "closest_waypoint_cost.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">Closest waypoint on a loop track: distance is not enough</text>
<rect x="80" y="110" width="1120" height="430" class="box"/>
<path d="M180 410 C290 250 460 250 600 390" stroke="#94a3b8" stroke-width="34" fill="none"/>
<path d="M185 455 C330 590 520 575 690 435" stroke="#94a3b8" stroke-width="34" fill="none"/>
<circle cx="420" cy="372" r="20" fill="#ef4444"/>
<line x1="420" y1="372" x2="510" y2="335" stroke="#16a34a" stroke-width="6"/>
<polygon points="532,326 495,323 509,356" fill="#16a34a"/>
<circle cx="455" cy="332" r="12" fill="#facc15"/>
<circle cx="455" cy="518" r="12" fill="#facc15"/>
<text x="485" y="328" class="small">near in distance, similar heading</text>
<text x="485" y="525" class="small">near in distance, opposite heading</text>
<line x1="455" y1="518" x2="365" y2="548" stroke="#ef4444" stroke-width="5"/>
<polygon points="342,556 379,559 366,526" fill="#ef4444"/>
<rect x="245" y="590" width="820" height="62" class="box orange"/>
<text x="288" y="630" class="small">cost = distance + heading_error_weight * abs(shortest_heading_error)</text>
</svg>
""",
        encoding="utf-8",
    )

    (DIAGRAM_DIR / "navigation_sensors_map.svg").write_text(
        f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
{svg_common}
<text x="48" y="62" class="title">Real vehicle navigation inputs for map-frame planning</text>
<rect x="95" y="130" width="230" height="120" class="box blue"/>
<text x="140" y="180" class="label">GNSS / RTK</text>
<text x="135" y="217" class="small">global position</text>
<rect x="385" y="130" width="230" height="120" class="box green"/>
<text x="450" y="180" class="label">IMU</text>
<text x="430" y="217" class="small">orientation, accel</text>
<rect x="675" y="130" width="230" height="120" class="box orange"/>
<text x="730" y="180" class="label">Wheel odom</text>
<text x="720" y="217" class="small">local motion</text>
<rect x="965" y="130" width="230" height="120" class="box purple"/>
<text x="1035" y="180" class="label">Map</text>
<text x="1005" y="217" class="small">road graph / lanes</text>
<path d="M210 260 C350 335 480 350 640 380" class="arrow"/>
<path d="M500 260 C555 310 605 340 640 380" class="arrow"/>
<path d="M790 260 C735 315 690 345 640 380" class="arrow"/>
<path d="M1080 260 C960 340 800 370 640 380" class="arrow"/>
<rect x="450" y="390" width="390" height="120" class="box"/>
<text x="505" y="440" class="label">Localization / Fusion</text>
<text x="500" y="477" class="small">ego pose in map frame</text>
<path d="M645 520 V600" class="arrow"/>
<rect x="430" y="610" width="430" height="70" class="box red"/>
<text x="475" y="654" class="small">Planner needs reliable T_map_base</text>
</svg>
""",
        encoding="utf-8",
    )


def main() -> None:
    ensure_dirs()
    copy_track_preview()
    generate_local_segment_capture()
    generate_terminal_assets()
    generate_svg_assets()


if __name__ == "__main__":
    main()
