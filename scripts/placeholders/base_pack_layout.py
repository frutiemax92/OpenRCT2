#!/usr/bin/env python3
"""Extracts the layout of the base graphics pack (AST-002): which image slots the game's UI uses, and their size and
draw offsets, so placeholder sprites fit the existing window layouts.

Only image metadata is read: the element table of g1.dat (width, height, offsets; never pixel data) and the image
sizes of the OpenRCT2 g2 sprites. The result, resources/base/layout.json, is committed; building the game and
generating the pack (base_pack.py) never need RCT or OpenRCT2 files.

Slots: every SPR_* name of the g1/g2 range referenced by the UI and interface code, plus the unnamed slots that
follow it (animation frames, up to MAX_FRAMES), plus the whole g2 range. Sprite-font glyphs are excluded: text is
drawn with TrueType fonts (AST-003).

Usage: base_pack_layout.py --g1 <RCT2>/Data/g1.dat [--output resources/base/layout.json]
"""

import argparse
import bisect
import json
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sprite_ids  # noqa: E402

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "resources", "base", "layout.json")
G2_MANIFEST = os.path.join(REPO_ROOT, "resources", "g2", "sprites.json")

# Code whose sprites make up the user interface.
UI_SOURCE_ROOTS = [
    "src/openrct2-ui",
    "src/openrct2/interface",
    "src/openrct2/localisation",
    "src/openrct2/management",
    "src/openrct2/scripting",
]
EXCLUDED_PREFIXES = ("SPR_CHAR", "SPR_FONTS", "SPR_TEMP")
MAX_FRAMES = 32

STORED_ELEMENT = struct.Struct("<IhhhhHH")  # StoredG1Element
FLAG_IS_PALETTE = 1 << 3


def referenced_names(root):
    names = set()
    for source_root in UI_SOURCE_ROOTS:
        for dirpath, dirnames, filenames in os.walk(os.path.join(root, source_root)):
            dirnames.sort()
            for name in sorted(filenames):
                if name.endswith((".cpp", ".h", ".hpp")):
                    with open(os.path.join(dirpath, name), "r", encoding="utf-8", errors="replace") as f:
                        names.update(re.findall(r"\bSPR_[A-Z0-9_]+\b", f.read()))
    return names


def read_g1_elements(path, count):
    with open(path, "rb") as f:
        num_entries, _ = struct.unpack("<II", f.read(8))
        if num_entries < count:
            raise RuntimeError("%s has %d entries, expected at least %d" % (path, num_entries, count))
        table = f.read(STORED_ELEMENT.size * count)
    return [STORED_ELEMENT.unpack_from(table, i * STORED_ELEMENT.size) for i in range(count)]


def png_size(path):
    with open(path, "rb") as f:
        header = f.read(24)
    if header[:8] != b"\x89PNG\r\n\x1a\n":
        raise RuntimeError("%s is not a PNG" % path)
    return struct.unpack(">II", header[16:24])


def g1_slots(values, names):
    """Maps slot -> (name, frame) for the referenced g1 names and their unnamed frames."""
    g1_end = values["SPR_G1_END"]
    named = sorted(set(v for v in values.values() if v <= g1_end))
    slots = {}
    for name in sorted(names):
        if name not in values or name.startswith(EXCLUDED_PREFIXES):
            continue
        start = values[name]
        if start >= g1_end:
            continue
        following = named[bisect.bisect_right(named, start)]
        span = following - start if following - start <= MAX_FRAMES else 1
        for frame in range(span):
            slots.setdefault(start + frame, (name, frame))
    return slots


def build_layout(root, g1_path):
    values = sprite_ids.load(os.path.join(root, "src", "openrct2", "SpriteIds.h"))
    names_by_value = {}
    for name, value in values.items():
        names_by_value.setdefault(value, name)

    entries = []
    elements = read_g1_elements(g1_path, values["SPR_G1_END"])
    for slot, (name, frame) in sorted(g1_slots(values, referenced_names(root)).items()):
        _, width, height, x, y, flags, _ = elements[slot]
        if width <= 0 or height <= 0 or flags & FLAG_IS_PALETTE:
            continue
        entries.append({"slot": slot, "name": name, "frame": frame, "width": width, "height": height, "x": x, "y": y})

    with open(os.path.join(root, "resources", "g2", "sprites.json"), "r", encoding="utf-8") as f:
        g2 = json.load(f)
    g2_begin = values["SPR_G2_BEGIN"]
    if len(g2) != values["SPR_G2_END"] - g2_begin:
        raise RuntimeError("resources/g2/sprites.json does not match the SPR_G2 range")
    for index, sprite in enumerate(g2):
        width, height = png_size(os.path.join(root, "resources", "g2", sprite["path"]))
        slot = g2_begin + index
        entries.append({
            "slot": slot, "name": names_by_value.get(slot, "SPR_G2_SLOT_%d" % index), "frame": 0,
            "width": width, "height": height, "x": sprite.get("x", 0), "y": sprite.get("y", 0),
        })
    return entries


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--g1", required=True, help="path to RCT2's Data/g1.dat (only its element table is read)")
    parser.add_argument("--root", default=REPO_ROOT, help="repository root")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="layout file to write")
    args = parser.parse_args(argv)

    entries = build_layout(args.root, args.g1)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8", newline="\n") as f:
        f.write("[\n")
        f.write(",\n".join("  " + json.dumps(e, sort_keys=True) for e in entries))
        f.write("\n]\n")
    print("Wrote %d slot(s) to %s" % (len(entries), args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
