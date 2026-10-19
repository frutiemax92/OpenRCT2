#!/usr/bin/env python3
"""Unit tests for scripts/placeholders/objects.py and terrain_layout.py (base object pack)."""

import importlib.util
import io
import json
import os
import re
import struct
import sys
import tempfile
import unittest
import zipfile

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
PLACEHOLDERS = os.path.join(ROOT, "scripts", "placeholders")
sys.path.insert(0, PLACEHOLDERS)


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(PLACEHOLDERS, name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


objects = load("objects")
terrain_layout = load("terrain_layout")

FILES = objects.build()
COMMITTED = os.path.join(ROOT, "data", "object", "golf")
SURFACE_C = os.path.join(ROOT, "src", "openrct2", "paint", "tile_element", "Paint.Surface.cpp")


def open_object(path):
    z = zipfile.ZipFile(io.BytesIO(FILES[path]))
    return z, json.loads(z.read("object.json"))


def png_size(data):
    return struct.unpack(">II", data[16:24])


def png_pixels(data):
    """RGBA rows of an 8-bit RGBA, non-interlaced PNG written by pixelart.Image (filter 0 on every row)."""
    import zlib

    width, height = png_size(data)
    pos, idat = 8, b""
    while pos < len(data):
        length = struct.unpack(">I", data[pos:pos + 4])[0]
        kind = data[pos + 4:pos + 8]
        if kind == b"IDAT":
            idat += data[pos + 8:pos + 8 + length]
        pos += 12 + length
    raw = zlib.decompress(idat)
    stride = 1 + 4 * width
    return [[tuple(raw[y * stride + 1 + 4 * x:y * stride + 5 + 4 * x]) for x in range(width)] for y in range(height)]


class SlopeTests(unittest.TestCase):
    def test_image_offsets_match_the_surface_painter(self):
        with open(SURFACE_C, "r", encoding="utf-8") as f:
            text = f.read()
        body = re.search(r"Byte97B444\[\] = \{(.*?)\};", text, re.S).group(1)
        table = [int(v) for v in re.findall(r"\d+", body)]
        self.assertEqual(table, objects.SLOPE_IMAGE_OFFSETS)
        for offset in range(objects.SURFACE_IMAGES_PER_ENTRY):
            self.assertEqual(offset, table[objects.slope_for_image_offset(offset)], offset)

    def test_corner_heights(self):
        self.assertEqual({1: 0, 2: 0, 4: 0, 8: 0}, objects.corner_heights(0))
        self.assertEqual({1: 1, 2: 0, 4: 0, 8: 0}, objects.corner_heights(objects.N))
        # Steep: the corner opposite the lowered one is two steps up.
        steep = objects.corner_heights(objects.STEEP | objects.E | objects.S | objects.W)
        self.assertEqual({1: 0, 2: 1, 4: 2, 8: 1}, steep)

    def test_flat_tile_is_a_64x32_diamond_at_the_tile_origin(self):
        image, x, y = objects.surface_image(0, (10, 200, 10), "FWY", "normal")
        self.assertEqual((64, 32, -32, 0), (image.width, image.height, x, y))
        self.assertEqual(0, image.get(0, 0)[3])  # corners of the box are transparent
        self.assertEqual(255, image.get(32, 16)[3])  # centre is filled

    def test_raised_corners_move_up_one_land_step(self):
        # S (top corner) up: the image starts 16 px above the origin.
        image, _, y = objects.surface_image(objects.S, (10, 200, 10), "", "normal")
        self.assertEqual((-16, 48), (y, image.height))
        # N (bottom corner) up: the bottom corner rises to y = 16.
        image, _, y = objects.surface_image(objects.N, (10, 200, 10), "", "normal")
        self.assertEqual((0, 16), (y, image.height))
        # Steep with S two steps up.
        image, _, y = objects.surface_image(objects.STEEP | objects.E | objects.S | objects.W, (1, 2, 3), "", "normal")
        self.assertEqual((-32, 64), (y, image.height))

    def test_sloped_faces_are_shaded_differently(self):
        flat, _, _ = objects.surface_image(0, (100, 100, 100), "", "normal")
        lit, _, _ = objects.surface_image(objects.E, (100, 100, 100), "", "normal")
        self.assertNotEqual(flat.get(32, 20), lit.get(20, 16))


class EdgeTests(unittest.TestCase):
    def test_band_fills_a_full_box(self):
        image = objects.edge_image({"width": 32, "height": 31, "x": -2, "y": -15}, (100, 100, 100))
        filled = sum(1 for row in image.pixels for p in row if p[3])
        self.assertGreater(filled, 32 * 12)
        self.assertEqual(0, image.get(0, 0)[3])  # above the rising top line

    def test_short_box_is_a_triangle(self):
        image = objects.edge_image({"width": 32, "height": 16, "x": -2, "y": 0}, (100, 100, 100))
        self.assertEqual(255, image.get(31, 15)[3])
        self.assertEqual(0, image.get(0, 0)[3])


class LayoutTests(unittest.TestCase):
    def test_reads_only_the_element_table(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "elements.dat")
            count = terrain_layout.EDGE_FIRST_IMAGE + terrain_layout.EDGE_IMAGE_COUNT
            with open(path, "wb") as f:
                f.write(struct.pack("<II", count, 0))
                for i in range(count):
                    f.write(struct.pack("<IhhhhHH", 0, 30 + i % 3, 20, -2, -15, 0, 0))
            layout = terrain_layout.build_layout(path)
        self.assertEqual(terrain_layout.EDGE_IMAGE_COUNT, len(layout["edge"]))
        self.assertEqual({"index": 0, "width": 30 + terrain_layout.EDGE_FIRST_IMAGE % 3, "height": 20, "x": -2,
                          "y": -15}, layout["edge"][0])

    def test_short_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "elements.dat")
            with open(path, "wb") as f:
                f.write(struct.pack("<II", 10, 0))
            with self.assertRaises(RuntimeError):
                terrain_layout.build_layout(path)

    def test_committed_layout_has_every_edge_image(self):
        with open(objects.TERRAIN_LAYOUT, "r", encoding="utf-8") as f:
            layout = json.load(f)
        self.assertEqual(list(range(terrain_layout.EDGE_IMAGE_COUNT)), [e["index"] for e in layout["edge"]])


class ObjectTests(unittest.TestCase):
    def test_every_object_is_generated(self):
        expected = {"terrain_surface/golf.terrain_surface.%s.parkobj" % s[0] for s in objects.SURFACES}
        expected |= {"terrain_edge/golf.terrain_edge.%s.parkobj" % e[0] for e in objects.EDGES}
        expected.add("climate/golf.climate.temperate.parkobj")
        self.assertEqual(expected, set(FILES))

    def test_manifests_are_our_own(self):
        for path in FILES:
            _, data = open_object(path)
            self.assertTrue(data["id"].startswith("golf."), path)
            self.assertEqual(objects.LICENCE, data["licence"])
            self.assertEqual(objects.AUTHORS, data["authors"])
            self.assertEqual(["custom"], data["sourceGame"])
            self.assertIn("en-GB", data["strings"]["name"])
            self.assertEqual(path.split("/")[0], data["objectType"])

    def test_images_are_in_the_archive(self):
        for path in FILES:
            z, data = open_object(path)
            for entry in data.get("images", []):
                width, height = png_size(z.read(entry["path"]))
                self.assertGreater(width, 0)
                self.assertGreater(height, 0)

    def test_terrain_surface_has_an_icon_and_three_entries(self):
        z, data = open_object("terrain_surface/golf.terrain_surface.fairway.parkobj")
        self.assertEqual(1 + 3 * objects.SURFACE_IMAGES_PER_ENTRY + 1, len(data["images"]))
        self.assertEqual({"normal": 0, "grid": 1, "underground": 2}, data["default"])
        flat = data["images"][1]
        self.assertEqual((-32, 0), (flat["x"], flat["y"]))
        self.assertEqual((64, 32), png_size(z.read(flat["path"])))
        map_colour = data["properties"]["mapColours"][0]
        self.assertTrue(objects.is_plain_index(map_colour), map_colour)

    def test_terrain_surface_ends_with_a_raw_square_texture(self):
        for name, label, _, _ in objects.SURFACES:
            z, data = open_object("terrain_surface/golf.terrain_surface.%s.parkobj" % name)
            self.assertTrue(data["properties"]["hasTexture"], name)
            texture = data["images"][-1]
            self.assertEqual("raw", texture["format"], name)
            self.assertEqual((0, 0), (texture["x"], texture["y"]))
            self.assertEqual((64, 64), png_size(z.read(texture["path"])), name)
            pixels = png_pixels(z.read(texture["path"]))
            self.assertTrue(all(p[3] == 255 for row in pixels for p in row), name)  # opaque, so no index 0

    def test_surfaces_have_distinct_colours(self):
        centres = set()
        for name, _, _, _ in objects.SURFACES:
            z, data = open_object("terrain_surface/golf.terrain_surface.%s.parkobj" % name)
            centres.add(png_pixels(z.read(data["images"][1]["path"]))[6][32])
        self.assertEqual(len(objects.SURFACES), len(centres))

    def test_terrain_edge_follows_the_layout(self):
        _, data = open_object("terrain_edge/golf.terrain_edge.rock_cliff.parkobj")
        with open(objects.TERRAIN_LAYOUT, "r", encoding="utf-8") as f:
            layout = json.load(f)
        images = data["images"]
        self.assertEqual(1 + len(layout["edge"]) + objects.EDGE_EXTRA_TUNNEL_IMAGES, len(images))
        for box, entry in zip(layout["edge"], images[1:]):
            self.assertEqual((box["x"], box["y"]), (entry["x"], entry["y"]))

    def test_climate_covers_the_season(self):
        _, data = open_object("climate/golf.climate.temperate.parkobj")
        self.assertEqual(
            ["march", "april", "may", "june", "july", "august", "september", "october"], list(data["weather"]))
        for month in data["weather"].values():
            # The engine rescales the weights with integer division: they must add up to kWeatherDistSize.
            self.assertEqual(objects.WEATHER_DIST_SIZE, sum(month["distribution"].values()))

    def test_weather_size_matches_the_engine(self):
        with open(os.path.join(ROOT, "src", "openrct2", "world", "Weather.h"), "r", encoding="utf-8") as f:
            match = re.search(r"kWeatherDistSize = (\d+);", f.read())
        self.assertEqual(objects.WEATHER_DIST_SIZE, int(match.group(1)))

    def test_images_use_only_plain_palette_colours(self):
        # Object images are imported by exact match: any other colour would become transparent.
        plain = set(objects.plain_palette(objects.load_palette()).values())
        for path in FILES:
            z, data = open_object(path)
            for entry in data.get("images", []):
                for row in png_pixels(z.read(entry["path"])):
                    for r, g, b, a in row:
                        if a:
                            self.assertIn((r, g, b), plain, "%s %s" % (path, entry["path"]))

    def test_load_palette(self):
        palette = objects.load_palette()
        self.assertEqual(256, len(palette))
        self.assertEqual((71, 27, 0), palette[46])  # stored as B, G, R, A

    def test_plain_indices_exclude_special_and_remap_ranges(self):
        for index in (0, 9, 46, 57, 202, 213, 230, 239, 243, 254, 255):
            self.assertFalse(objects.is_plain_index(index), index)
        for index in (10, 45, 58, 201, 214, 229, 240, 242):
            self.assertTrue(objects.is_plain_index(index), index)

    def test_plain_ranges_match_the_importer(self):
        with open(os.path.join(ROOT, "src", "openrct2", "drawing", "ImageImporter.cpp"), "r", encoding="utf-8") as f:
            text = f.read()
        body = text[text.index("ImageImporter::GetPaletteIndexType(int32_t paletteIndex)"):]
        body = body[:body.index("return PaletteIndexType::normal;")]
        for index in range(256):
            special = False
            for low, high in re.findall(r"paletteIndex >= (\d+) && paletteIndex <= (\d+)", body):
                special |= int(low) <= index <= int(high)
            special |= index <= int(re.search(r"paletteIndex <= (\d+)\)", body).group(1))
            special |= index == int(re.search(r"paletteIndex == (\d+)\)", body).group(1))
            self.assertEqual(not special, objects.is_plain_index(index), index)

    def test_quantise_keeps_transparency(self):
        image = objects.Image(2, 1, (0, 0, 0), alpha=0)
        image.set(1, 0, (250, 10, 10))
        objects.quantise(image, objects.load_palette())
        self.assertEqual(0, image.get(0, 0)[3])
        self.assertIn(image.get(1, 0)[:3], set(objects.plain_palette(objects.load_palette()).values()))

    def test_nearest_palette_index(self):
        palette = {10: (0, 0, 0), 11: (250, 250, 250), 12: (0, 200, 0)}
        self.assertEqual(12, objects.nearest_palette_index((10, 190, 10), palette))
        self.assertEqual(10, objects.nearest_palette_index((5, 5, 5), palette))

    def test_archives_are_reproducible_and_stored(self):
        self.assertEqual(FILES, objects.build())
        for content in FILES.values():
            z = zipfile.ZipFile(io.BytesIO(content))
            self.assertEqual("object.json", z.infolist()[0].filename)
            for info in z.infolist():
                self.assertEqual(zipfile.ZIP_STORED, info.compress_type)

    def test_committed_objects_are_up_to_date(self):
        for path, content in FILES.items():
            with open(os.path.join(COMMITTED, path), "rb") as f:
                self.assertEqual(content, f.read(), path + " is stale: run scripts/placeholders/objects.py")

    def test_main_writes_the_objects(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(0, objects.main(["--output", tmp]))
            for path in FILES:
                self.assertTrue(os.path.isfile(os.path.join(tmp, path)), path)


if __name__ == "__main__":
    unittest.main()
