#!/usr/bin/env python3
"""Base object pack generator: the game's own placeholder objects (AST-002, AST-004).

Generates the objects the map and the scenario editor need by default, so that no RCT or OpenRCT2 object is required:

  golf.terrain_surface.<name>   one per golf surface of generate.py (grass, tee, fairway, green, ...)
  golf.terrain_edge.rock_cliff  terrain edge
  golf.climate.temperate        climate
  golf.stream.creek             water stream
  golf.building.club_house      3x2 golf club (4 views of a flat-shaded box)
  golf.street.road              two-way road (asphalt texture, centre and edge lines)
  golf.street.one_way           one-way road (asphalt texture, edge lines and arrows)

Terrain surfaces use the existing terrain_surface format: an icon, then 3 entries (normal, grid, underground) of 19
images, one per surface slope, drawn as flat-shaded isometric tiles from the tile geometry, then the 64x64 tileable
square placeholder texture of generate.py that the procedural terrain renderer samples (hasTexture). Terrain edges use the
existing terrain_edge format; each edge image fills the sprite box recorded in resources/base/terrain_layout.json
(terrain_layout.py) with a wall band along the tile side. These formats are replaced by golf_terrain_surface and
golf_terrain_edge in Phase 1 (§6.5, §6.7).

Object images are imported by exact colour match against the game palette (ImageImporter's StandardPalette), so every
pixel is quantised to the nearest palette colour that is neither special nor remappable.

Output is byte-reproducible: pure Python images (pixelart.py) and stored archives with fixed timestamps (parkobj.py).

Usage: objects.py [--output DIR]   (default: data/object/golf)
"""

import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate import EDGES, SURFACES, edge_texture, surface_icon, surface_texture  # noqa: E402
from glb import car_glb, cart_glb, deer_glb, duck_glb, golfer_glb  # noqa: E402
from parkobj import pack  # noqa: E402
from pixelart import Image, draw_text, shade, text_colour, text_size  # noqa: E402

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "data", "object", "golf")
TERRAIN_LAYOUT = os.path.join(REPO_ROOT, "resources", "base", "terrain_layout.json")
IMAGE_IMPORTER_H = os.path.join(REPO_ROOT, "src", "openrct2", "drawing", "ImageImporter.h")

# Smoothing priority of each surface (TER-041): the higher one bleeds over the lower where they meet.
SMOOTHING_PRIORITY = {
    "grass": 10, "rough": 20, "deep_rough": 25, "woods": 30, "rocks": 35, "fairway": 40, "tee": 50,
    "mini_putt_carpet": 55, "green": 60, "sand": 70,
}

# Gameplay properties of each surface (TER-042): roughness, bounciness, distanceFactor, accuracyFactor, puttable, hazard.
SURFACE_GOLF = {
    "green": (0.05, 0.30, 1.00, 1.00, True, False),
    "tee": (0.10, 0.40, 1.00, 1.00, False, False),
    "fairway": (0.15, 0.45, 1.00, 1.00, False, False),
    "grass": (0.35, 0.35, 0.90, 0.90, False, False),
    "rough": (0.45, 0.30, 0.85, 0.85, False, False),
    "deep_rough": (0.75, 0.20, 0.60, 0.65, False, False),
    "sand": (0.90, 0.05, 0.55, 0.75, False, True),
    "woods": (0.80, 0.25, 0.50, 0.50, False, False),
    "rocks": (0.30, 0.75, 0.70, 0.60, False, False),
    "mini_putt_carpet": (0.05, 0.30, 1.00, 1.00, True, False),
}

AUTHORS =["Open Golf Simulator contributors"]
LICENCE = "CC0-1.0"

# Surface slope bits (kTileSlope*CornerUp) and the steep (diagonal) flag.
N, E, S, W = 1, 2, 4, 8
STEEP = 0x10
OPPOSITE = {N: S, S: N, E: W, W: E}

# Image offset of each surface slope, as Byte97B444 in paint/tile_element/Paint.Surface.cpp (slope -> image offset).
SLOPE_IMAGE_OFFSETS = [
    0, 2, 1, 3, 8, 10, 9, 11, 4, 6, 5, 7, 12, 14, 13, 15, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 16, 0, 18, 15, 0,
]
SURFACE_IMAGES_PER_ENTRY = 19
LAND_STEP_PX = 16  # screen pixels per land height step

# Screen position (x, y) of each map corner of a flat tile at view rotation 0, relative to the image origin.
# S (map 0,0) is the top corner, E (32,0) the left one, W (0,32) the right one and N (32,32) the bottom one.
CORNER_SCREEN = {S: (0, 0), E: (-32, 16), W: (32, 16), N: (0, 32)}
CORNER_WORLD = {S: (0, 0), E: (32, 0), W: (0, 32), N: (32, 32)}

EDGE_EXTRA_TUNNEL_IMAGES = 4  # the terrain edge format has 4 more tunnel images after the 44 of the layout


def slope_for_image_offset(offset):
    """The surface slope drawn by an image offset; offset 15 is the steep slope with the S corner two steps up."""
    if offset == 15:
        return STEEP | E | S | W
    return SLOPE_IMAGE_OFFSETS.index(offset) if offset < 15 else SLOPE_IMAGE_OFFSETS.index(offset, 16)


def corner_heights(slope):
    """Height in land steps of each corner for a surface slope."""
    heights = {c: (1 if slope & c else 0) for c in (N, E, S, W)}
    if slope & STEEP:
        down = next(c for c in (N, E, S, W) if not slope & c)
        heights[OPPOSITE[down]] = 2
    return heights


def screen_corners(slope):
    heights = corner_heights(slope)
    return {c: (CORNER_SCREEN[c][0], CORNER_SCREEN[c][1] - LAND_STEP_PX * heights[c]) for c in (N, E, S, W)}, heights


def inside_triangle(px, py, a, b, c):
    def side(p, q):
        return (q[0] - p[0]) * (py - p[1]) - (q[1] - p[1]) * (px - p[0])

    d1, d2, d3 = side(a, b), side(b, c), side(c, a)
    has_neg = d1 < 0 or d2 < 0 or d3 < 0
    has_pos = d1 > 0 or d2 > 0 or d3 > 0
    return not (has_neg and has_pos)


def triangle_light(corners, heights):
    """Brightness of a surface triangle: lit from the top-left of the screen, like the original terrain."""
    (ax, ay), (bx, by), (cx, cy) = (CORNER_WORLD[c] for c in corners)
    az, bz, cz = (heights[c] * LAND_STEP_PX for c in corners)
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    if nz < 0:
        nx, ny, nz = -nx, -ny, -nz
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    # Light comes from the map's -x/-y side (the top-left of the screen at rotation 0) and above.
    lx, ly, lz = -0.45, -0.45, 0.77
    return 0.72 + 0.34 * ((nx * lx + ny * ly + nz * lz) / length)


def surface_triangles(slope):
    """The tile split into two triangles, along the diagonal whose corners have the same height when possible."""
    _, heights = screen_corners(slope)
    if heights[E] == heights[W] and heights[N] != heights[S]:
        return [(S, E, W), (N, W, E)]
    return [(S, E, N), (S, N, W)]


def surface_image(slope, colour, label, kind):
    """One isometric tile image and its draw offset. kind: "normal", "grid" (outlined) or "underground" (dark)."""
    points, heights = screen_corners(slope)
    xs = [p[0] for p in points.values()]
    ys = [p[1] for p in points.values()]
    left, top = min(xs), min(ys)
    width, height = max(xs) - left, max(ys) - top
    base = shade(colour, 0.45) if kind == "underground" else colour
    image = Image(max(1, width), max(1, height), (0, 0, 0), alpha=0)
    if width == 0 or height == 0:
        return image, left, top

    shifted = {c: (p[0] - left, p[1] - top) for c, p in points.items()}
    triangles = [(t, shade(base, triangle_light(t, heights))) for t in surface_triangles(slope)]
    for y in range(height):
        for x in range(width):
            px, py = x + 0.5, y + 0.5
            for corners, fill in triangles:
                if inside_triangle(px, py, *(shifted[c] for c in corners)):
                    image.set(x, y, fill)
                    break

    if kind == "grid":
        outline = shade(base, 0.55)
        order = [S, W, N, E]
        for i, c in enumerate(order):
            (x0, y0), (x1, y1) = shifted[c], shifted[order[(i + 1) % 4]]
            steps = max(abs(x1 - x0), abs(y1 - y0))
            for s in range(steps + 1):
                x = min(width - 1, int(x0 + (x1 - x0) * s / steps))
                y = min(height - 1, int(y0 + (y1 - y0) * s / steps))
                if image.get(x, y)[3]:
                    image.set(x, y, outline)

    if slope == 0 and kind != "underground":
        tw, th = text_size(label, 1)
        draw_text(image, label, (width - tw) // 2, (height - th) // 2, text_colour(colour), 1)
    return image, left, top


def edge_image(box, colour):
    """A terrain edge image: a wall band hanging from the tile side, filling its sprite box."""
    width, height = box["width"], box["height"]
    image = Image(width, height, (0, 0, 0), alpha=0)
    # Pieces right of the origin hang from the bottom-right tile side (rising to the right, darker); pieces left of it
    # hang from the bottom-left side (falling to the right, lit).
    rising = box["x"] >= -2
    fill = shade(colour, 0.8 if rising else 1.05)
    top_line = shade(fill, 1.3)
    drop = width // 2
    thickness = height - drop
    for x in range(width):
        top = (width - 1 - x) // 2 if rising else x // 2
        # Boxes too short for a full band hold the triangular pieces under sloped sides: fill to the bottom.
        bottom = top + thickness if thickness >= 4 else height
        for y in range(top, min(height, bottom)):
            image.set(x, y, top_line if y == top else fill)
    return image


def edge_icon(label, colour):
    icon = Image(32, 32, colour)
    for i in range(32):
        icon.set(i, 0, shade(colour, 0.55))
        icon.set(i, 31, shade(colour, 0.55))
        icon.set(0, i, shade(colour, 0.55))
        icon.set(31, i, shade(colour, 0.55))
    for y in range(8, 32, 8):
        for x in range(1, 31):
            icon.set(x, y, shade(colour, 0.8))
    tw, th = text_size(label, 1)
    draw_text(icon, label, (32 - tw) // 2, (32 - th) // 2, text_colour(colour), 1)
    return icon


def load_palette(path=IMAGE_IMPORTER_H):
    """{palette index: RGB} of the palette object images are imported with (StandardPalette, stored as B, G, R, A)."""
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    body = text[text.index("StandardPalette = { {"):]
    body = re.sub(r"//[^\n]*", "", body[:body.index("} };")])
    entries = re.findall(r"\{\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\s*\}", body)
    if len(entries) != 256:
        raise RuntimeError("expected 256 StandardPalette entries in %s, found %d" % (path, len(entries)))
    return {i: (int(r), int(g), int(b)) for i, (b, g, r, _) in enumerate(entries)}


def is_plain_index(index):
    """Palette indices an image can use as is: not special (transparent, animated) and not remapped by colours
    (ImageImporter::GetPaletteIndexType)."""
    return not (index <= 9 or 230 <= index <= 239 or index == 255 or 243 <= index <= 254 or 202 <= index <= 213
                or 46 <= index <= 57)


def plain_palette(palette):
    return {i: c for i, c in palette.items() if is_plain_index(i)}


def quantise(image, palette):
    """Replaces the colour of every opaque pixel with the nearest plain palette colour."""
    plain = plain_palette(palette)
    cache = {}
    for row in image.pixels:
        for x, (r, g, b, a) in enumerate(row):
            if a == 0:
                continue
            if (r, g, b) not in cache:
                cache[(r, g, b)] = plain[nearest_palette_index((r, g, b), plain)]
            row[x] = cache[(r, g, b)] + (255,)
    return image


def nearest_palette_index(colour, palette):
    return min(palette, key=lambda i: (sum((a - b) ** 2 for a, b in zip(palette[i], colour)), i))


def manifest(object_id, object_type, name, properties, images=None):
    data = {
        "id": object_id,
        "authors": AUTHORS,
        "version": "1.0",
        "licence": LICENCE,
        "sourceGame": ["custom"],
        "objectType": object_type,
        "properties": properties,
    }
    if images is not None:
        data["images"] = images
    data["strings"] = {"name": {"en-GB": name}}
    return data


def object_files(data, images):
    files = {"object.json": (json.dumps(data, indent=4) + "\n").encode("utf-8")}
    files.update(images)
    return files


def terrain_surface(name, label, colour, description, palette):
    images = {"images/icon.png": quantise(surface_icon(label, colour), palette).png()}
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    for kind in ("normal", "grid", "underground"):
        for offset in range(SURFACE_IMAGES_PER_ENTRY):
            image, x, y = surface_image(slope_for_image_offset(offset), colour, label, kind)
            path = "images/%s_%02d.png" % (kind, offset)
            images[path] = quantise(image, palette).png()
            table.append({"path": path, "x": x, "y": y})
    images["images/texture.png"] = quantise(surface_texture(label, colour), palette).png()
    table.append({"path": "images/texture.png", "x": 0, "y": 0, "format": "raw"})
    map_colour = nearest_palette_index(colour, plain_palette(palette))
    properties = {
        "price": 100,
        "mapColours": [map_colour, map_colour],
        "rotations": 1,
        "hasTexture": True,  # the last image is the square texture
        "smoothing": {"priority": SMOOTHING_PRIORITY[name]},
    }
    roughness, bounciness, distance, accuracy, puttable, hazard = SURFACE_GOLF[name]
    properties.update({
        "canGrow": name in ("grass", "fairway", "green", "tee", "rough", "deep_rough"),
        "roughness": roughness,
        "bounciness": bounciness,
        "buildCost": 100,
        "removeCost": 20,
        "lie": {"distanceFactor": distance, "accuracyFactor": accuracy},
        "isHazard": hazard,
        "isPuttable": puttable,
    })
    data = manifest("golf.terrain_surface." + name, "terrain_surface", description, properties, table)
    data["default"] = {"normal": 0, "grid": 1, "underground": 2}
    return object_files(data, images)


def terrain_edge(name, label, colour, description, layout, palette):
    images = {"images/icon.png": quantise(edge_icon(label, colour), palette).png()}
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    boxes = list(layout["edge"])
    boxes += [dict(boxes[36 + i], index=len(layout["edge"]) + i) for i in range(EDGE_EXTRA_TUNNEL_IMAGES)]
    for box in boxes:
        path = "images/edge_%02d.png" % box["index"]
        images[path] = quantise(edge_image(box, colour), palette).png()
        table.append({"path": path, "x": box["x"], "y": box["y"]})
    images["images/texture.png"] = quantise(edge_texture(label, colour), palette).png()
    table.append({"path": "images/texture.png", "x": 0, "y": 0, "format": "raw"})
    data = manifest("golf.terrain_edge." + name, "terrain_edge", description, {"hasTexture": True}, table)
    return object_files(data, images)


STREAM_FRAMES = 4
STREAM_TEXTURE_SIZE = 64


def stream_icon(colour):
    image = Image(32, 32, (0, 0, 0), 0)
    for y in range(8, 24):
        for x in range(2, 30):
            image.set(x, y, shade(colour, 1.1 if (x // 4 + y // 4) % 2 else 0.9))
    return image


def stream_frame(colour, frame):
    # Diagonal ripples drifting one band per frame so the strip loops; the texture tiles in both directions.
    size = STREAM_TEXTURE_SIZE
    period = size // STREAM_FRAMES
    image = Image(size, size, colour)
    for y in range(size):
        for x in range(size):
            band = ((x + y) - frame * (period // STREAM_FRAMES * 4)) % period
            image.set(x, y, shade(colour, 1.2 if band < period // 4 else 1.0))
    return image


def stream(name, description, colour, palette):
    images = {"images/icon.png": quantise(stream_icon(colour), palette).png()}
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    for frame in range(STREAM_FRAMES):
        path = "images/frame_%d.png" % frame
        images[path] = quantise(stream_frame(colour, frame), palette).png()
        table.append({"path": path, "x": 0, "y": 0, "format": "raw"})
    properties = {
        "buildCost": 150,
        "removeCost": 30,
        "width": 12,
        "image": {"frames": STREAM_FRAMES, "frameDurationTicks": 8},
        "isHazard": True,
    }
    data = manifest("golf.stream." + name, "stream", description, properties, table)
    return object_files(data, images)


STREET_TEXTURE_SIZE = 64
STREET_MARKING_COLOURS = {
    "centre": (255, 255, 255),
    "edge": (255, 220, 0),
    "arrow": (120, 200, 255),
}


def street_texture():
    """Asphalt for a road running along the x axis, with fine deterministic grain; it tiles in both directions."""
    size = STREET_TEXTURE_SIZE
    image = Image(size, size, (0x58, 0x58, 0x5C))
    for y in range(size):
        for x in range(size):
            grain = (x * 7 + y * 13 + (x * y) % 5) % 6
            image.set(x, y, shade((0x58, 0x58, 0x5C), 0.92 + 0.03 * grain))
    return image


def street_markings(one_way):
    """The mask of the markings, transparent where there are none: an edge line along each verge, and either a dashed
    centre line (two-way) or an arrow pointing along +x (one-way)."""
    size = STREET_TEXTURE_SIZE
    image = Image(size, size, (0, 0, 0), 0)
    # A texel is about a third of a world unit across the road, and the road is drawn a pixel per unit: the lines are four
    # texels thick so they show.
    for x in range(size):
        for y in (3, 4, 5, 6, size - 7, size - 6, size - 5, size - 4):
            image.set(x, y, STREET_MARKING_COLOURS["edge"])
    mid = size // 2
    if one_way:
        for k in range(18):
            for t in range(-2, 2):
                image.set(22 + k, mid + t, STREET_MARKING_COLOURS["arrow"])
        for k in range(1, 9):
            for t in range(-1, 2):
                image.set(40 - k, mid - 2 - k + t, STREET_MARKING_COLOURS["arrow"])
                image.set(40 - k, mid + 1 + k + t, STREET_MARKING_COLOURS["arrow"])
    else:
        for x in range(size):
            if (x // 8) % 2 == 0:
                for t in range(-2, 2):
                    image.set(x, mid + t, STREET_MARKING_COLOURS["centre"])
    return image


def street_icon(one_way):
    image = Image(32, 32, (0, 0, 0), 0)
    image.fill_rect(2, 8, 28, 16, (0x58, 0x58, 0x5C))
    if one_way:
        image.fill_rect(8, 15, 14, 2, STREET_MARKING_COLOURS["arrow"])
        image.fill_rect(20, 12, 2, 8, STREET_MARKING_COLOURS["arrow"])
    else:
        for x in range(2, 30, 6):
            image.fill_rect(x, 15, 3, 2, STREET_MARKING_COLOURS["centre"])
    return image


def street(name, description, one_way, speed_limit, palette):
    images = {
        "images/icon.png": quantise(street_icon(one_way), palette).png(),
        "images/road.png": quantise(street_texture(), palette).png(),
        "images/markings.png": quantise(street_markings(one_way), palette).png(),
    }
    table = [
        {"path": "images/icon.png", "x": 0, "y": 0},
        {"path": "images/road.png", "x": 0, "y": 0, "format": "raw"},
        {"path": "images/markings.png", "x": 0, "y": 0, "format": "raw"},
    ]
    properties = {
        "buildCost": 120,
        "removeCost": 30,
        "oneWay": one_way,
        "speedLimit": speed_limit,
    }
    data = manifest("golf.street." + name, "street", description, properties, table)
    return object_files(data, images)


BUILDING_HEIGHT = 48


def building_view(footprint, rotation, colour):
    """An isometric box covering the footprint, seen from a camera rotation. Returns (image, x offset, y offset)."""
    fx, fy = footprint if rotation % 2 == 0 else footprint[::-1]
    # One tile is 64 pixels wide and 32 high on screen; the box rises BUILDING_HEIGHT pixels.
    width = (fx + fy) * 32
    height = (fx + fy) * 16 + BUILDING_HEIGHT
    image = Image(width, height, (0, 0, 0), 0)
    for y in range(height):
        for x in range(width):
            # Position relative to the top corner of the roof diamond, which sits at the top of the image.
            u = (x - fy * 32) / 32.0
            v = y / 16.0
            # The roof diamond is the set of (a, b) tile coordinates in [0, fx] x [0, fy].
            a = (v + u) / 2.0
            b = (v - u) / 2.0
            if 0 <= a < fx and 0 <= b < fy:
                image.set(x, y, shade(colour, 1.15))
                continue
            # Walls: the roof diamond translated down by up to BUILDING_HEIGHT.
            for drop in range(0, BUILDING_HEIGHT, 1):
                v2 = (y - drop) / 16.0
                a2 = (v2 + u) / 2.0
                b2 = (v2 - u) / 2.0
                if 0 <= a2 < fx and 0 <= b2 < fy:
                    image.set(x, y, shade(colour, 0.9 if u < 0 else 0.7))
                    break
    return image, -fy * 32, -height + (fx + fy) * 8


def path_addition_icon(colour):
    image = Image(32, 32, (0, 0, 0), 0)
    image.fill_rect(6, 10, 20, 12, shade(colour, 1.0))
    image.fill_rect(8, 12, 16, 8, shade(colour, 1.2))
    return image


def path_addition_view(colour, broken=False, full=False):
    image = Image(16, 16, (0, 0, 0), 0)
    c = shade(colour, 0.7 if broken else (1.3 if full else 1.0))
    image.fill_rect(2, 4, 12, 8, c)
    return image


def path_addition(name, description, render_as, is_bench, is_bin, is_lamp, colour, palette, price=300, is_sign=False, is_ball_washer=False):
    images = {"images/icon.png": quantise(path_addition_icon(colour), palette).png()}
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    # 4 intact
    for rot in range(4):
        path = "images/intact_%d.png" % rot
        images[path] = quantise(path_addition_view(colour, False, False), palette).png()
        table.append({"path": path, "x": -8, "y": -8})
    # 4 broken
    for rot in range(4):
        path = "images/broken_%d.png" % rot
        images[path] = quantise(path_addition_view(colour, True, False), palette).png()
        table.append({"path": path, "x": -8, "y": -8})
    # 4 full (if bin)
    if is_bin:
        for rot in range(4):
            path = "images/full_%d.png" % rot
            images[path] = quantise(path_addition_view(colour, False, True), palette).png()
            table.append({"path": path, "x": -8, "y": -8})

    properties = {
        "renderAs": render_as,
        "price": price,
        "isBench": is_bench,
        "isBin": is_bin,
        "isLamp": is_lamp,
        "isSign": is_sign,
        "isBallWasher": is_ball_washer,
        "cursor": "CURSOR_BENCH_DOWN" if is_bench else ("CURSOR_BIN_DOWN" if is_bin else ("CURSOR_LAMPPOST_DOWN" if is_lamp else "CURSOR_STATUE_DOWN")),
    }
    data = manifest("golf.path_addition." + name, "path_addition", description, properties, table)
    return object_files(data, images)


# Footpath surface layout, as PathPaintGetBaseImage in paint/tile_element/Paint.Path.cpp: 51 flat images (0-15 by edge
# mask NE SE SW NW, the rest corner variants of the same shapes), then 16 sloped ones (4 rises of 1-4 quarter steps
# times 4 directions).
FOOTPATH_FLAT_IMAGES = 51
FOOTPATH_SLOPE_IMAGES = 16
FOOTPATH_SLOPE_DIRECTIONS = [S | W, N | W, N | E, S | E]  # slope direction 0..3 = SW, NW, NE, SE side up
FOOTPATH_ARM = (8, 24)  # the path is half a tile wide (PTH-001): 16 of the 32 units, centred
# Railings and bridge images the paint code can index (bridge images start at 37 and reach about 55 beyond that).
FOOTPATH_RAILING_IMAGES = 100


def footpath_flat_image(edges, colour, paved):
    """A flat tile (S is the top corner, E the left one) with the path drawn as a centre square plus an arm to each connected edge (all of it when paved)."""
    image = Image(64, 32, (0, 0, 0), alpha=0)
    lo, hi = FOOTPATH_ARM
    for y in range(32):
        for x in range(64):
            sx, sy = x + 0.5 - 32, y + 0.5
            wx, wy = sy - sx / 2, sy + sx / 2
            if not (0 <= wx < 32 and 0 <= wy < 32):
                continue
            centre = lo <= wx < hi and lo <= wy < hi
            arms = ((edges & 1) and wx < lo and lo <= wy < hi) or ((edges & 2) and wy >= hi and lo <= wx < hi) \
                or ((edges & 4) and wx >= hi and lo <= wy < hi) or ((edges & 8) and wy < lo and lo <= wx < hi)
            if paved or centre or arms:
                border = paved is False and not (lo + 1 <= wx < hi - 1 or lo + 1 <= wy < hi - 1)
                image.set(x, y, shade(colour, 0.8 if border else 1.0 + ((x + y) % 3) * 0.04))
    return image


def footpath_surface_icon(colour):
    icon = Image(32, 32, shade(colour, 0.6))
    icon.fill_rect(4, 4, 24, 24, colour)
    return icon


def footpath_surface(name, description, colour, palette, queue=False):
    images = {"images/icon.png": quantise(footpath_surface_icon(colour), palette).png()}
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    for offset in range(FOOTPATH_FLAT_IMAGES):
        # Image offsets above 15 draw corner variants of the 16 edge shapes: the placeholder keeps the same shape.
        image = footpath_flat_image(offset & 15 if offset < 16 else 15, colour, offset >= 16)
        path = "images/flat_%02d.png" % offset
        images[path] = quantise(image, palette).png()
        table.append({"path": path, "x": -32, "y": 0})
    for rise in range(4):
        for direction in range(4):
            image, x, y = surface_image(FOOTPATH_SLOPE_DIRECTIONS[direction], colour, "", "normal")
            path = "images/slope_%d_%d.png" % (rise + 1, direction)
            images[path] = quantise(image, palette).png()
            table.append({"path": path, "x": x, "y": y})
    properties = {"isQueue": queue}
    return object_files(manifest("golf.footpath_surface." + name, "footpath_surface", description, properties, table), images)


def footpath_railings(name, description, palette):
    """Invisible railings: every image is a single transparent pixel, there are as many as the paint code can index."""
    blank = quantise(Image(1, 1, (0, 0, 0), alpha=0), palette).png()
    images = {"images/blank.png": blank}
    table = [{"path": "images/blank.png", "x": 0, "y": 0}] * (FOOTPATH_RAILING_IMAGES + 1)
    properties = {"supportType": "pole", "scrollingMode": 0, "hasSupportImages": False}
    return object_files(manifest("golf.footpath_railings." + name, "footpath_railings", description, properties, table), images)


def peep_animations(name, description, peep_type, animations, palette):
    """Sprite animations the staff code still reads (bounds, preview image). Staff are drawn as 3D models like the golfers,
    so every animation is the same single transparent frame in the 4 view directions."""
    blank = quantise(Image(1, 1, (0, 0, 0), alpha=0), palette).png()
    images = {"images/blank.png": blank}
    table = [{"path": "images/blank.png", "x": 0, "y": 0}] * 5
    group = {"animations": {a: {"offset": 0, "sequence": [0]} for a in animations}}
    data = manifest("golf.peep_animations." + name, "peep_animations", description,
                    {"peepType": peep_type, "noRandomPlacement": True}, table)
    data["animationGroups"] = [group]
    return object_files(data, images)


def building_icon(colour):
    image = Image(32, 32, (0, 0, 0), 0)
    image.fill_rect(6, 12, 20, 14, shade(colour, 0.9))
    image.fill_rect(4, 8, 24, 5, shade(colour, 1.2))
    return image


def building(name, description, kind, footprint, colour, services, palette,
             build_cost=25000, remove_cost=5000, upkeep=300, capacity=20):
    images = {"images/icon.png": quantise(building_icon(colour), palette).png()}
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    for rotation in range(4):
        view, x, y = building_view(footprint, rotation, colour)
        path = "images/view_%d.png" % rotation
        images[path] = quantise(view, palette).png()
        table.append({"path": path, "x": x, "y": y})
    properties = {
        "kind": kind,
        "footprint": list(footprint),
        "height": BUILDING_HEIGHT,
        "entrance": {"tile": [footprint[0] // 2, 0], "edge": 3},
        "buildCost": build_cost,
        "removeCost": remove_cost,
        "upkeepPerMonth": upkeep,
        "services": services,
        "capacity": capacity,
    }
    data = manifest("golf.building." + name, "building", description, properties, table)
    return object_files(data, images)


# A mild climate with frequent sun: golfers play from spring to autumn. Temperatures in °C. The weather weights of each
# month add up to WEATHER_DIST_SIZE.
CLIMATE_MONTHS = [
    # month, base temperature, random bias, sunny, partially cloudy, cloudy, rain, heavy rain, thunder
    ("march", 9, 9, 6, 6, 5, 4, 1, 1),
    ("april", 12, 9, 7, 6, 5, 3, 1, 1),
    ("may", 16, 9, 9, 6, 4, 3, 1, 0),
    ("june", 20, 8, 11, 5, 3, 2, 1, 1),
    ("july", 23, 8, 12, 5, 3, 1, 1, 1),
    ("august", 22, 8, 11, 5, 3, 2, 1, 1),
    ("september", 18, 9, 8, 6, 5, 3, 1, 0),
    ("october", 13, 9, 6, 6, 6, 3, 2, 0),
]
WEATHER_DIST_SIZE = 23  # kWeatherDistSize: the weights of a month must add up to it (world/Weather.h)


def draw_disc(image, cx, cy, r, colour):
    for y in range(max(0, cy - r), min(image.height, cy + r + 1)):
        for x in range(max(0, cx - r), min(image.width, cx + r + 1)):
            if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r:
                image.set(x, y, colour)


def small_scenery_view(colour, is_cylinder=False):
    image = Image(24, 24, (0, 0, 0), 0)
    if is_cylinder:
        draw_disc(image, 12, 12, 8, shade(colour, 1.0))
        draw_disc(image, 12, 10, 6, shade(colour, 1.3))
    else:
        image.fill_rect(4, 6, 16, 12, shade(colour, 1.0))
        image.fill_rect(6, 8, 12, 8, shade(colour, 1.3))
    return image


def small_scenery(name, description, shape_str, height, price, removal_price,
                  collision, colour, palette, rotatable=False, distance_sign=False):
    images = {}
    table = []
    num_views = 4 if rotatable else 1
    is_cylinder = collision.get("shape") == "cylinder"
    for r in range(num_views):
        path = "images/view_%d.png" % r
        images[path] = quantise(small_scenery_view(colour, is_cylinder), palette).png()
        table.append({"path": path, "x": -12, "y": -12})

    properties = {
        "price": price,
        "removalPrice": removal_price,
        "height": height,
        "cursor": "CURSOR_STATUE_DOWN",
        "shape": shape_str,
        "isRotatable": rotatable,
        "collision": collision,
    }
    if distance_sign:
        properties["isDistanceSign"] = True
    data = manifest("golf.scenery_small." + name, "scenery_small", description, properties, table)
    return object_files(data, images)


def wall_view(colour, rising=False):
    image = Image(32, 24, (0, 0, 0), 0)
    c = shade(colour, 0.9 if rising else 1.1)
    image.fill_rect(2, 6, 28, 12, c)
    image.fill_rect(4, 4, 24, 4, shade(colour, 1.3))
    return image


def wall(name, description, height, price, collision, colour, palette):
    images = {}
    table = []
    for i in range(6):
        path = "images/view_%d.png" % i
        images[path] = quantise(wall_view(colour, i % 2 == 1), palette).png()
        table.append({"path": path, "x": -16, "y": -12})

    properties = {
        "price": price,
        "height": height,
        "cursor": "CURSOR_FENCE_DOWN",
        "collision": collision,
    }
    data = manifest("golf.wall." + name, "wall", description, properties, table)
    return object_files(data, images)


def vehicle_icon(colour):
    image = Image(32, 32, (0, 0, 0), 0)
    image.fill_rect(6, 12, 20, 10, shade(colour, 1.0))
    image.fill_rect(10, 8, 12, 6, shade(colour, 1.3))
    return image


def vehicle_model(name, description, kind, seats, max_speed, accel,
                  contact_half_len, contact_half_width, engine_sound, horn_sound,
                  collision, colour, palette):
    images = {
        "images/icon.png": quantise(vehicle_icon(colour), palette).png(),
        "model.glb": car_glb() if kind == "car" else cart_glb(),
    }
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    properties = {
        "kind": kind,
        "model": "model.glb",
        "seats": seats,
        "maxSpeed": max_speed,
        "acceleration": accel,
        "contactPoints": {
            "halfLength": contact_half_len,
            "halfWidth": contact_half_width,
        },
        "sounds": {
            "engine": engine_sound,
            "horn": horn_sound,
        },
        "collision": collision,
    }
    data = manifest("golf.vehicle_model." + name, "vehicle_model", description, properties, table)
    return object_files(data, images)


def golfer_model(name, description, palette):
    images = {
        "images/icon.png": quantise(vehicle_icon((220, 180, 50)), palette).png(),
        "model.glb": golfer_glb(),
    }
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    properties = {
        "model": "model.glb",
        "triangleBudget": 800,
        "materialSlots": 3,
        "clips": [
            "idle", "walk", "sit_bench", "talk", "swing_full",
            "swing_putt", "cheer", "groan", "celebrate", "rage_quit"
        ],
        "collision": {
            "shape": "cylinder",
            "height": 32,
            "radius": 6,
            "restitution": 0.3,
            "friction": 0.5
        }
    }
    data = manifest("golf.golfer_model." + name, "golfer_model", description, properties, table)
    return object_files(data, images)


def animal(name, description, habitat, group_size, spawn_rate, sound, palette):
    images = {
        "images/icon.png": quantise(vehicle_icon((100, 180, 80)), palette).png(),
        "model.glb": duck_glb() if name == "duck" else deer_glb(),
    }
    table = [{"path": "images/icon.png", "x": 0, "y": 0}]
    properties = {
        "model": "model.glb",
        "triangleBudget": 600,
        "habitat": habitat,
        "minGroupSize": group_size[0],
        "maxGroupSize": group_size[1],
        "spawnRate": spawn_rate,
        "sound": sound,
        "clips": [
            "idle", "walk", "run", "flee"
        ],
        "collision": {
            "shape": "cylinder",
            "height": 12,
            "radius": 6,
            "restitution": 0.2,
            "friction": 0.6
        }
    }
    data = manifest("golf.animal." + name, "animal", description, properties, table)
    return object_files(data, images)


def climate():
    weather = {}
    for month, base, bias, sunny, partly, cloudy, rain, heavy, thunder in CLIMATE_MONTHS:
        weather[month] = {
            "baseTemperature": base,
            "randomBias": bias,
            "distribution": {
                "sunny": sunny, "partiallyCloudy": partly, "cloudy": cloudy, "rain": rain, "heavyRain": heavy,
                "thunder": thunder,
            },
            "distRotation": 0,
        }
    data = manifest(
        "golf.climate.temperate", "climate", "Temperate", {"coldItemTempThreshold": 10, "warmItemTempThreshold": 21})
    data["scriptName"] = "temperate"
    data["weather"] = weather
    return object_files(data, {})


def build(layout=None, palette=None):
    """Returns {relative path: bytes} of the .parkobj files."""
    if layout is None:
        with open(TERRAIN_LAYOUT, "r", encoding="utf-8") as f:
            layout = json.load(f)
    if palette is None:
        palette = load_palette()
    out = {}
    for name, label, colour, description in SURFACES:
        out["terrain_surface/golf.terrain_surface.%s.parkobj" % name] = pack(
            terrain_surface(name, label, colour, description, palette))
    for name, label, colour, description in EDGES:
        out["terrain_edge/golf.terrain_edge.%s.parkobj" % name] = pack(
            terrain_edge(name, label, colour, description, layout, palette))
    out["climate/golf.climate.temperate.parkobj"] = pack(climate())
    out["street/golf.street.road.parkobj"] = pack(street("road", "Two-way road", False, 40, palette))
    out["street/golf.street.one_way.parkobj"] = pack(street("one_way", "One-way road", True, 30, palette))
    out["stream/golf.stream.creek.parkobj"] = pack(stream("creek", "Creek", (0x3A, 0x7B, 0xC8), palette))
    out["building/golf.building.club_house.parkobj"] = pack(
        building("club_house", "Golf club", "golf_club", (3, 2), (0xB0, 0x7A, 0x4A),
                 ["food", "drink", "toilet", "pro_shop"], palette,
                 build_cost=25000, remove_cost=5000, upkeep=300, capacity=20))
    out["building/golf.building.food_truck.parkobj"] = pack(
        building("food_truck", "Food truck", "food_truck", (1, 1), (0xD0, 0x40, 0x30),
                 ["food", "drink"], palette,
                 build_cost=5000, remove_cost=1000, upkeep=80, capacity=5))
    out["building/golf.building.cottage.parkobj"] = pack(
        building("cottage", "Golf cottage", "cottage", (2, 2), (0xC0, 0x90, 0x60),
                 ["rest"], palette,
                 build_cost=40000, remove_cost=8000, upkeep=150, capacity=4))
    out["building/golf.building.driving_range_kiosk.parkobj"] = pack(
        building("driving_range_kiosk", "Driving range kiosk", "driving_range_kiosk", (1, 1), (0x40, 0x80, 0x50),
                 ["pro_shop"], palette,
                 build_cost=8000, remove_cost=1500, upkeep=100, capacity=10))
    out["building/golf.building.cart_rental.parkobj"] = pack(
        building("cart_rental", "Cart rental", "cart_rental", (2, 1), (0x60, 0x70, 0x90),
                 [], palette,
                 build_cost=15000, remove_cost=3000, upkeep=200, capacity=12))
    out["building/golf.building.restroom.parkobj"] = pack(
        building("restroom", "Restroom", "restroom", (1, 1), (0x80, 0x80, 0x80),
                 ["toilet"], palette,
                 build_cost=6000, remove_cost=1200, upkeep=60, capacity=8))
    out["footpath_surface/golf.footpath_surface.gravel.parkobj"] = pack(
        footpath_surface("gravel", "Gravel path", (0xB8, 0xA0, 0x78), palette))
    out["footpath_surface/golf.footpath_surface.tee_queue.parkobj"] = pack(
        footpath_surface("tee_queue", "Tee waiting path", (0xA0, 0x88, 0x60), palette, queue=True))
    out["footpath_railings/golf.footpath_railings.none.parkobj"] = pack(
        footpath_railings("none", "No railings", palette))
    out["peep_animations/golf.peep_animations.greenskeeper.parkobj"] = pack(
        peep_animations("greenskeeper", "Greenskeeper", "handyman",
                        ["walking", "watchRide", "hanging", "staffMower", "staffSweep", "drowning", "staffWatering",
                         "staffEmptyBin"], palette))
    out["path_addition/golf.path_addition.bench.parkobj"] = pack(
        path_addition("bench", "Park bench", "bench", True, False, False, (0x8B, 0x5A, 0x2B), palette, price=50))
    out["path_addition/golf.path_addition.bin.parkobj"] = pack(
        path_addition("bin", "Litter bin", "bin", False, True, False, (0x4A, 0x70, 0x4A), palette, price=30))
    out["path_addition/golf.path_addition.lamp.parkobj"] = pack(
        path_addition("lamp", "Lamp post", "lamp", False, False, True, (0x20, 0x20, 0x30), palette, price=40))
    out["path_addition/golf.path_addition.ball_washer.parkobj"] = pack(
        path_addition("ball_washer", "Ball washer", "ball_washer", False, False, False, (0x20, 0x60, 0x20), palette, price=60, is_ball_washer=True))
    out["path_addition/golf.path_addition.path_sign.parkobj"] = pack(
        path_addition("path_sign", "Path sign", "sign", False, False, False, (0x70, 0x4A, 0x20), palette, price=40, is_sign=True))
    out["scenery_small/golf.scenery_small.bumper.parkobj"] = pack(
        small_scenery("bumper", "Mini-putt bumper", "1/4", 1, 50, 10,
                      {"shape": "cylinder", "radius": 6, "height": 8, "restitution": 0.90, "friction": 0.10},
                      (0xE0, 0x30, 0x30), palette, rotatable=False))
    out["scenery_small/golf.scenery_small.ramp.parkobj"] = pack(
        small_scenery("ramp", "Mini-putt ramp", "4/4", 1, 80, 20,
                      {"shape": "box", "footprintX": 32, "footprintY": 32, "height": 8, "restitution": 0.50, "friction": 0.20},
                      (0x30, 0x60, 0xC0), palette, rotatable=True))
    out["scenery_small/golf.scenery_small.distance_sign.parkobj"] = pack(
        small_scenery("distance_sign", "Driving range distance sign", "1/4", 14, 40, 8,
                      {"shape": "cylinder", "radius": 2, "height": 14, "restitution": 0.50, "friction": 0.20},
                      (0xF0, 0xF0, 0xF0), palette, rotatable=False, distance_sign=True))
    out["wall/golf.wall.fishnet.parkobj"] = pack(
        wall("fishnet", "Driving range fishnet", 48, 60,
             {"shape": "box", "height": 48, "restitution": 0.05, "friction": 0.30},
             (0x40, 0x70, 0x40), palette))
    out["wall/golf.wall.low_wall.parkobj"] = pack(
        wall("low_wall", "Mini-putt border wall", 1, 20,
             {"shape": "box", "height": 8, "restitution": 0.85, "friction": 0.15},
             (0xA0, 0x50, 0x30), palette))
    out["vehicle_model/golf.vehicle_model.golf_cart.parkobj"] = pack(
        vehicle_model("golf_cart", "Golf cart", "golf_cart", 2, 2.20, 0.08,
                      10.0, 6.0, "golf.cart.engine", "golf.cart.horn",
                      {"shape": "box", "footprintX": 20, "footprintY": 12, "height": 14, "restitution": 0.35, "friction": 0.40},
                      (0xF0, 0xF0, 0xF0), palette))
    out["vehicle_model/golf.vehicle_model.car.parkobj"] = pack(
        vehicle_model("car", "Visitor car", "car", 4, 2.00, 0.06,
                      12.0, 6.0, "golf.car.engine", "golf.car.horn",
                      {"shape": "box", "footprintX": 24, "footprintY": 12, "height": 16, "restitution": 0.35, "friction": 0.40},
                      (0x30, 0x50, 0xA0), palette))
    out["golfer_model/golf.golfer_model.standard.parkobj"] = pack(
        golfer_model("standard", "Standard Golfer", palette))
    out["animal/golf.animal.duck.parkobj"] = pack(
        animal("duck", "Mallard Duck", ["water", "grass"], (2, 6), 15, "duck", palette))
    out["animal/golf.animal.deer.parkobj"] = pack(
        animal("deer", "White-tailed Deer", ["rough", "woods"], (1, 4), 5, "birdChirp", palette))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="output directory")
    args = parser.parse_args(argv)
    files = build()
    for path, content in sorted(files.items()):
        full = os.path.join(args.output, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "wb") as f:
            f.write(content)
    print("Wrote %d object(s) to %s" % (len(files), args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
