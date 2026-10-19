#!/usr/bin/env python3
"""Extracts the sprite boxes of terrain edge images, so placeholder terrain edges (objects.py) line up with the terrain
painter.

Only image metadata is read: width, height and draw offsets from the element table of RCT2's g1.dat, never pixel data.
The result, resources/base/terrain_layout.json, is committed; generating the objects never needs RCT files.

Terrain surface images need no extraction: their shapes follow from the tile geometry (see objects.py).

Usage: terrain_layout.py --g1 <RCT2>/Data/g1.dat [--output resources/base/terrain_layout.json]
"""

import argparse
import json
import os
import struct
import sys

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "resources", "base", "terrain_layout.json")

STORED_ELEMENT = struct.Struct("<IhhhhHH")  # StoredG1Element

# First edge image of RCT2's rock terrain edge, and how many edge images a terrain edge object has before its extra
# tunnel images (see GetEdgeImage and kTunnels in paint/tile_element/Paint.Surface.cpp).
EDGE_FIRST_IMAGE = 1831
EDGE_IMAGE_COUNT = 44


def read_elements(path, first, count):
    with open(path, "rb") as f:
        num_entries, _ = struct.unpack("<II", f.read(8))
        if num_entries < first + count:
            raise RuntimeError("%s has %d entries, expected at least %d" % (path, num_entries, first + count))
        f.seek(8 + first * STORED_ELEMENT.size)
        table = f.read(STORED_ELEMENT.size * count)
    return [STORED_ELEMENT.unpack_from(table, i * STORED_ELEMENT.size) for i in range(count)]


def build_layout(g1_path):
    edges = []
    for index, (_, width, height, x, y, _, _) in enumerate(read_elements(g1_path, EDGE_FIRST_IMAGE, EDGE_IMAGE_COUNT)):
        edges.append({"index": index, "width": width, "height": height, "x": x, "y": y})
    return {"edge": edges}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--g1", required=True, help="path to RCT2's Data/g1.dat (only its element table is read)")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="layout file to write")
    args = parser.parse_args(argv)

    layout = build_layout(args.g1)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8", newline="\n") as f:
        f.write('{\n  "edge": [\n')
        f.write(",\n".join("    " + json.dumps(e, sort_keys=True) for e in layout["edge"]))
        f.write("\n  ]\n}\n")
    print("Wrote %d edge box(es) to %s" % (len(layout["edge"]), args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
