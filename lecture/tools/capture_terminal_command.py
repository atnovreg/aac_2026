#!/usr/bin/env python3
"""Run a command and render its terminal output as a PNG capture."""

import argparse
import os
import re
import subprocess
import textwrap
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def strip_ansi(text):
    return ANSI_RE.sub("", text)


def load_font(size):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
        "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


def wrap_lines(text, width):
    lines = []
    for line in text.splitlines() or [""]:
        if not line:
            lines.append("")
            continue
        wrapped = textwrap.wrap(
            line,
            width=width,
            replace_whitespace=False,
            drop_whitespace=False,
            break_long_words=True,
            break_on_hyphens=False,
        )
        lines.extend(wrapped or [""])
    return lines


def render_capture(title, command, body, output_png, max_width, font_size):
    font = load_font(font_size)
    bold_font = load_font(font_size)
    line_height = int(font_size * 1.45)
    margin = 28
    title_height = 40
    char_width = max(8, int(font.getlength("M")))
    wrap_width = max(40, (max_width - margin * 2) // char_width)

    prompt_lines = wrap_lines(f"$ {command}", wrap_width)
    output_lines = wrap_lines(body.rstrip(), wrap_width)
    lines = prompt_lines + output_lines
    max_lines = 34
    if len(lines) > max_lines:
        lines = lines[: max_lines - 2] + ["", "... output truncated ..."]

    image_height = title_height + margin * 2 + line_height * max(1, len(lines))
    image = Image.new("RGB", (max_width, image_height), "#101317")
    draw = ImageDraw.Draw(image)

    draw.rectangle((0, 0, max_width, title_height), fill="#2f343a")
    draw.ellipse((16, 14, 28, 26), fill="#ff5f57")
    draw.ellipse((36, 14, 48, 26), fill="#ffbd2e")
    draw.ellipse((56, 14, 68, 26), fill="#28c840")
    draw.text((86, 10), title, font=bold_font, fill="#e8edf2")

    y = title_height + margin
    for idx, line in enumerate(lines):
        color = "#82aaff" if idx < len(prompt_lines) else "#e8edf2"
        draw.text((margin, y), line, font=font, fill=color)
        y += line_height

    output_png.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_png)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--title", required=True)
    parser.add_argument("--command", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cwd", default=None)
    parser.add_argument("--timeout", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--font-size", type=int, default=24)
    args = parser.parse_args()

    result = subprocess.run(
        args.command,
        shell=True,
        cwd=args.cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=args.timeout,
        check=False,
    )
    body = strip_ansi(result.stdout)
    if result.returncode:
        body = f"{body.rstrip()}\n\n[exit code: {result.returncode}]\n"

    output_png = Path(args.output)
    output_txt = output_png.with_suffix(".txt")
    output_txt.parent.mkdir(parents=True, exist_ok=True)
    output_txt.write_text(f"$ {args.command}\n{body}", encoding="utf-8")

    render_capture(
        args.title,
        args.command,
        body,
        output_png,
        max_width=args.width,
        font_size=args.font_size,
    )


if __name__ == "__main__":
    main()
