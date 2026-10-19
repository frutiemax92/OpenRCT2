#!/usr/bin/env python3
"""Placeholder art generator (AST-004).

Generates the project's placeholder images reproducibly: same script, same bytes. Placeholders are deliberately
distinctive (flat colours, checker patterns, labels) so a developer can identify each texture, quadrant and slope
level at a glance. Pure Python (no Pillow) so the output does not depend on an imaging library version.

Texture space: u grows towards +x (right), v towards +y (down). A dark 3x3 mark sits at the texture origin
(u = 0, v = 0) to show orientation and tiling. Edges are named after map directions: -x (left), +y (bottom),
+x (right), -y (top), i.e. directions 0, 1, 2, 3.

Usage: generate.py [--output DIR]   (default: resources/placeholders)
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pixelart import *  # noqa: E402,F401,F403  (re-exported for the tests)
from pixelart import BLACK, Image, draw_label, shade, text_size  # noqa: E402

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "resources", "placeholders")

TEXTURE_SIZE = 64  # TER-030: square, power of two, tileable
ICON_SIZE = 32
CHECKER = 8

# Initial golf_terrain_surface set (§6.5, TER-042) plus the mini-putt carpet (MPT-001).
# (file name, label, RGB map colour, description)
SURFACES = [
    ("grass", "GRS", (0x7F, 0xAE, 0x4A), "Grass: unmanaged land"),
    ("tee", "TEE", (0x2F, 0x8F, 0x5B), "Tee box"),
    ("fairway", "FWY", (0x3F, 0xA3, 0x4D), "Fairway"),
    ("green", "GRN", (0x6F, 0xD3, 0x6A), "Green (puttable)"),
    ("rough", "RGH", (0x4F, 0x7F, 0x2F), "Rough"),
    ("deep_rough", "DRG", (0x2F, 0x5A, 0x20), "Deep rough"),
    ("sand", "SND", (0xE3, 0xCF, 0x8A), "Sand (bunker, hazard)"),
    ("woods", "WDS", (0x28, 0x46, 0x2A), "Woods"),
    ("rocks", "RCK", (0x8A, 0x8A, 0x8A), "Rocks"),
    ("mini_putt_carpet", "MPT", (0x2E, 0x7F, 0xD0), "Mini-putt carpet (puttable)"),
]

# golf_terrain_edge set (§6.7).
EDGES = [
    ("rock_cliff", "EDGE", (0x8C, 0x6E, 0x50), "Rock cliff"),
]

# Quadrant debug colours, as in specifications/tile_texture.png. Keyed by the tile edge each triangle touches.
QUADRANTS = [
    ("-X", (0x2C, 0xA6, 0xB5)),  # direction 0, left edge
    ("+Y", (0xB2, 0x2B, 0x2C)),  # direction 1, bottom edge
    ("+X", (0x63, 0xB4, 0x30)),  # direction 2, right edge
    ("-Y", (0x40, 0x2A, 0xB5)),  # direction 3, top edge
]

# Slope level debug colours: corner offset or height difference of 0..4 quarter steps (§6.2).
SLOPE_LEVELS = [
    ("0Q", (0xF0, 0xF0, 0xF0)),
    ("1Q", (0x5B, 0xC0, 0xEB)),
    ("2Q", (0x9B, 0xC5, 0x3D)),
    ("3Q", (0xFA, 0xA4, 0x3A)),
    ("4Q", (0xE5, 0x59, 0x34)),
]

def draw_origin_mark(image, colour):
    image.fill_rect(0, 0, 3, 3, shade(colour, 0.35))


def checker(size, colour):
    image = Image(size, size, colour)
    dark = shade(colour, 0.88)
    for y in range(size):
        for x in range(size):
            if ((x // CHECKER) + (y // CHECKER)) % 2:
                image.set(x, y, dark)
    return image


def surface_texture(label, colour):
    image = checker(TEXTURE_SIZE, colour)
    draw_label(image, label, TEXTURE_SIZE // 2, TEXTURE_SIZE // 2, colour, 2)
    draw_origin_mark(image, colour)
    return image


def surface_icon(label, colour):
    image = Image(ICON_SIZE, ICON_SIZE, colour)
    border = shade(colour, 0.5)
    for i in range(ICON_SIZE):
        for x, y in ((i, 0), (i, ICON_SIZE - 1), (0, i), (ICON_SIZE - 1, i)):
            image.set(x, y, border)
    draw_label(image, label, ICON_SIZE // 2, ICON_SIZE // 2, colour, 2 if text_size(label, 2)[0] <= ICON_SIZE - 6 else 1)
    return image


def edge_texture(label, colour):
    # Horizontal strata (4 px bands, period 16) so vertical tiling and the up direction are visible.
    image = Image(TEXTURE_SIZE, TEXTURE_SIZE, colour)
    for y in range(TEXTURE_SIZE):
        factor = (1.0, 0.85, 1.1, 0.75)[(y // 4) % 4]
        for x in range(TEXTURE_SIZE):
            image.set(x, y, shade(colour, factor))
    draw_label(image, label, TEXTURE_SIZE // 2, TEXTURE_SIZE // 2, colour, 2)
    draw_origin_mark(image, colour)
    return image


def quadrant_of(x, y, size):
    """Quadrant (index into QUADRANTS) of pixel centre (x, y): split by the two diagonals (TER-020)."""
    u = (x + 0.5) / size - 0.5
    v = (y + 0.5) / size - 0.5
    if abs(u) >= abs(v):
        return 0 if u < 0 else 2
    return 3 if v < 0 else 1


def quadrant_texture():
    size = TEXTURE_SIZE
    image = Image(size, size)
    for y in range(size):
        for x in range(size):
            image.set(x, y, QUADRANTS[quadrant_of(x, y, size)][1])
    centres = {"-X": (10, 32), "+Y": (32, 54), "+X": (54, 32), "-Y": (32, 10)}
    for label, colour in QUADRANTS:
        cx, cy = centres[label]
        draw_label(image, label, cx, cy, colour, 1)
    draw_origin_mark(image, (0x80, 0x80, 0x80))
    return image


def slope_level_texture(label, colour):
    image = Image(TEXTURE_SIZE, TEXTURE_SIZE, colour)
    draw_label(image, label, TEXTURE_SIZE // 2, TEXTURE_SIZE // 2, colour, 3)
    draw_origin_mark(image, colour)
    return image


def hex_colour(colour):
    return "#%02x%02x%02x" % colour


def build():
    """Returns {relative path: bytes} for every generated file, manifest included."""
    files = {}
    manifest = {"generator": "scripts/placeholders/generate.py", "licence": "CC0-1.0", "images": []}

    def add(path, image, kind, label, colour, description):
        files[path] = image.png()
        manifest["images"].append({
            "path": path, "kind": kind, "label": label, "colour": hex_colour(colour),
            "size": [image.width, image.height], "description": description,
        })

    for name, label, colour, description in SURFACES:
        add("terrain/%s.png" % name, surface_texture(label, colour), "terrain_surface", label, colour, description)
        add("terrain/%s_icon.png" % name, surface_icon(label, colour), "terrain_surface_icon", label, colour, description)
    for name, label, colour, description in EDGES:
        add("terrain_edge/%s.png" % name, edge_texture(label, colour), "terrain_edge", label, colour, description)
    add("debug/quadrants.png", quadrant_texture(), "debug_quadrants", "-X +Y +X -Y", (0x80, 0x80, 0x80),
        "Tile quadrants -X, +Y, +X, -Y (TER-020, tile_texture.png)")
    for label, colour in SLOPE_LEVELS:
        add("debug/slope_%s.png" % label.lower(), slope_level_texture(label, colour), "debug_slope_level", label, colour,
            "Slope level %s (quarter steps, §6.2)" % label)

    files["manifest.json"] = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    return files


def write(output, files):
    for path, content in sorted(files.items()):
        full = os.path.join(output, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "wb") as f:
            f.write(content)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="output directory")
    args = parser.parse_args(argv)
    files = build()
    write(args.output, files)
    print("Wrote %d file(s) to %s" % (len(files), args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
