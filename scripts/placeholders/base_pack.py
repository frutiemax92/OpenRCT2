#!/usr/bin/env python3
"""Generates the base graphics pack sources (AST-002) from resources/base/layout.json.

Writes one placeholder PNG per slot into resources/base/images and resources/base/sprites.json, which the build
turns into base.gx with `sprite build`. A placeholder is a box of the slot's size with a darker border, filled with
a colour derived from the sprite name (animation frames share a hue and get lighter), labelled with as much of the
name as fits.

Usage: base_pack.py [--layout FILE] [--output DIR]
"""

import argparse
import colorsys
import json
import os
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pixelart import Image, draw_text, shade, text_colour, text_size  # noqa: E402

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_LAYOUT = os.path.join(REPO_ROOT, "resources", "base", "layout.json")
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "resources", "base")


def words(name):
    parts = name.split("+")[0].split("_")
    return [p for p in parts if p not in ("SPR", "G2", "")]


def label_candidates(name, frame):
    w = words(name)
    suffix = [str(frame)] if frame else []
    candidates = [
        " ".join(w + suffix),
        " ".join([p for p in w if not p.isdigit()] + suffix),
        "".join(p[0] for p in w) + (" " + suffix[0] if suffix else ""),
        "".join(p[0] for p in w),
    ]
    if suffix:
        candidates.append(suffix[0])
    return [c for c in candidates if c]


def label_for(name, frame, width, height):
    """Longest candidate that fits inside the border, or "" when none does."""
    if height < 7:
        return ""
    for candidate in label_candidates(name, frame):
        if text_size(candidate, 1)[0] <= width - 2:
            return candidate
    return ""


def colour_for(name, frame):
    hue = (zlib.crc32(name.encode("utf-8")) % 360) / 360.0
    r, g, b = colorsys.hsv_to_rgb(hue, 0.55, 0.80)
    return shade((int(round(r * 255)), int(round(g * 255)), int(round(b * 255))), 1.0 + 0.06 * min(frame, 5))


def placeholder(entry):
    width, height = entry["width"], entry["height"]
    colour = colour_for(entry["name"], entry["frame"])
    image = Image(width, height, colour)
    border = shade(colour, 0.55)
    for x in range(width):
        image.set(x, 0, border)
        image.set(x, height - 1, border)
    for y in range(height):
        image.set(0, y, border)
        image.set(width - 1, y, border)
    label = label_for(entry["name"], entry["frame"], width, height)
    if label:
        tw, th = text_size(label, 1)
        draw_text(image, label, (width - tw) // 2, (height - th) // 2, text_colour(colour), 1)
    return image


def image_path(entry):
    return "images/%05d_%s.png" % (entry["slot"], entry["name"].lower().replace("+", "_"))


def build(layout):
    """Returns {relative path: bytes} for the images and sprites.json."""
    files = {}
    sprites = []
    previous = -1
    for entry in layout:
        if entry["slot"] <= previous:
            raise ValueError("layout slots must increase (slot %d)" % entry["slot"])
        previous = entry["slot"]
        path = image_path(entry)
        files[path] = placeholder(entry).png()
        sprites.append({"slot": entry["slot"], "path": path, "x": entry["x"], "y": entry["y"]})
    files["sprites.json"] = (
        "[\n" + ",\n".join("  " + json.dumps(s, sort_keys=True) for s in sprites) + "\n]\n"
    ).encode("utf-8")
    return files


def write(output, files):
    for path, content in sorted(files.items()):
        full = os.path.join(output, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "wb") as f:
            f.write(content)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--layout", default=DEFAULT_LAYOUT, help="layout file (from base_pack_layout.py)")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="output directory")
    args = parser.parse_args(argv)
    with open(args.layout, "r", encoding="utf-8") as f:
        layout = json.load(f)
    files = build(layout)
    write(args.output, files)
    print("Wrote %d file(s) to %s" % (len(files), args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
