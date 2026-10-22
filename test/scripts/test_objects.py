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
        expected.add("stream/golf.stream.creek.parkobj")
        expected |= {"street/golf.street.road.parkobj", "street/golf.street.one_way.parkobj"}
        expected.add("building/golf.building.club_house.parkobj")
        expected.add("building/golf.building.food_truck.parkobj")
        expected.add("building/golf.building.cottage.parkobj")
        expected.add("building/golf.building.driving_range_kiosk.parkobj")
        expected.add("building/golf.building.cart_rental.parkobj")
        expected.add("building/golf.building.restroom.parkobj")
        expected |= {"footpath_surface/golf.footpath_surface.gravel.parkobj",
                     "footpath_surface/golf.footpath_surface.tee_queue.parkobj",
                     "footpath_railings/golf.footpath_railings.none.parkobj"}
        expected.add("peep_animations/golf.peep_animations.greenskeeper.parkobj")
        expected.add("path_addition/golf.path_addition.bench.parkobj")
        expected.add("path_addition/golf.path_addition.bin.parkobj")
        expected.add("path_addition/golf.path_addition.lamp.parkobj")
        expected.add("path_addition/golf.path_addition.ball_washer.parkobj")
        expected.add("path_addition/golf.path_addition.path_sign.parkobj")
        expected.add("scenery_small/golf.scenery_small.bumper.parkobj")
        expected.add("scenery_small/golf.scenery_small.ramp.parkobj")
        expected.add("wall/golf.wall.low_wall.parkobj")
        expected.add("wall/golf.wall.fishnet.parkobj")
        expected.add("scenery_small/golf.scenery_small.distance_sign.parkobj")
        expected.add("vehicle_model/golf.vehicle_model.golf_cart.parkobj")
        expected.add("vehicle_model/golf.vehicle_model.car.parkobj")
        expected.add("golfer_model/golf.golfer_model.standard.parkobj")
        expected.add("animal/golf.animal.duck.parkobj")
        expected.add("animal/golf.animal.deer.parkobj")
        self.assertEqual(expected, set(FILES))

    def test_streets_have_a_texture_and_a_markings_mask(self):
        for name, one_way in (("road", False), ("one_way", True)):
            _, data = open_object("street/golf.street.%s.parkobj" % name)
            self.assertEqual("street", data["objectType"])
            self.assertEqual(one_way, data["properties"]["oneWay"])
            self.assertGreater(data["properties"]["speedLimit"], 0)
            # The icon, the road texture and the markings mask; both are raw so the engine reads palette indices.
            self.assertEqual(3, len(data["images"]))
            self.assertTrue(all(i.get("format") == "raw" for i in data["images"][1:]))

    def test_golfer_model_properties(self):
        _, golfer = open_object("golfer_model/golf.golfer_model.standard.parkobj")
        self.assertEqual("golfer_model", golfer["objectType"])
        self.assertEqual("model.glb", golfer["properties"]["model"])
        self.assertEqual(800, golfer["properties"]["triangleBudget"])
        self.assertEqual(3, golfer["properties"]["materialSlots"])
        required_clips = [
            "idle", "walk", "sit_bench", "talk", "swing_full",
            "swing_putt", "cheer", "groan", "celebrate", "rage_quit"
        ]
        self.assertEqual(required_clips, golfer["properties"]["clips"])
        self.assertEqual("cylinder", golfer["properties"]["collision"]["shape"])

    def test_animal_properties(self):
        _, duck = open_object("animal/golf.animal.duck.parkobj")
        self.assertEqual("animal", duck["objectType"])
        self.assertEqual(600, duck["properties"]["triangleBudget"])
        self.assertEqual(["water", "grass"], duck["properties"]["habitat"])
        self.assertEqual(2, duck["properties"]["minGroupSize"])
        self.assertEqual(6, duck["properties"]["maxGroupSize"])
        self.assertEqual(15, duck["properties"]["spawnRate"])
        self.assertEqual(["idle", "walk", "run", "flee"], duck["properties"]["clips"])
        self.assertEqual("cylinder", duck["properties"]["collision"]["shape"])

    def test_vehicle_model_properties(self):
        _, cart = open_object("vehicle_model/golf.vehicle_model.golf_cart.parkobj")
        self.assertEqual("vehicle_model", cart["objectType"])
        self.assertEqual("golf_cart", cart["properties"]["kind"])
        self.assertEqual(2, cart["properties"]["seats"])
        self.assertEqual(2.20, cart["properties"]["maxSpeed"])
        self.assertEqual(0.08, cart["properties"]["acceleration"])
        self.assertEqual(10.0, cart["properties"]["contactPoints"]["halfLength"])
        self.assertEqual(6.0, cart["properties"]["contactPoints"]["halfWidth"])
        self.assertEqual("golf.cart.engine", cart["properties"]["sounds"]["engine"])
        self.assertEqual("golf.cart.horn", cart["properties"]["sounds"]["horn"])
        self.assertEqual("box", cart["properties"]["collision"]["shape"])

        _, car = open_object("vehicle_model/golf.vehicle_model.car.parkobj")
        self.assertEqual("vehicle_model", car["objectType"])
        self.assertEqual("car", car["properties"]["kind"])
        self.assertEqual(4, car["properties"]["seats"])
        self.assertEqual(2.00, car["properties"]["maxSpeed"])
        self.assertEqual(0.06, car["properties"]["acceleration"])
        self.assertEqual(12.0, car["properties"]["contactPoints"]["halfLength"])
        self.assertEqual(6.0, car["properties"]["contactPoints"]["halfWidth"])
        self.assertEqual("golf.car.engine", car["properties"]["sounds"]["engine"])
        self.assertEqual("golf.car.horn", car["properties"]["sounds"]["horn"])
        self.assertEqual("box", car["properties"]["collision"]["shape"])

    def test_mini_putt_obstacle_properties(self):
        _, bumper = open_object("scenery_small/golf.scenery_small.bumper.parkobj")
        self.assertEqual("scenery_small", bumper["objectType"])
        self.assertEqual("cylinder", bumper["properties"]["collision"]["shape"])
        self.assertEqual(0.9, bumper["properties"]["collision"]["restitution"])

        _, ramp = open_object("scenery_small/golf.scenery_small.ramp.parkobj")
        self.assertEqual("scenery_small", ramp["objectType"])
        self.assertEqual("box", ramp["properties"]["collision"]["shape"])
        self.assertEqual(4, len(ramp["images"]))

        _, wall_obj = open_object("wall/golf.wall.low_wall.parkobj")
        self.assertEqual("wall", wall_obj["objectType"])
        self.assertEqual("box", wall_obj["properties"]["collision"]["shape"])
        self.assertEqual(6, len(wall_obj["images"]))

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

    def test_surfaces_have_distinct_smoothing_priorities(self):
        priorities = set()
        for name, _, _, _ in objects.SURFACES:
            _, data = open_object("terrain_surface/golf.terrain_surface.%s.parkobj" % name)
            priorities.add(data["properties"]["smoothing"]["priority"])
        self.assertEqual(len(objects.SURFACES), len(priorities))
        self.assertEqual({s[0] for s in objects.SURFACES}, set(objects.SMOOTHING_PRIORITY))

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
        self.assertEqual(1 + len(layout["edge"]) + objects.EDGE_EXTRA_TUNNEL_IMAGES + 1, len(images))  # + texture
        self.assertTrue(data["properties"]["hasTexture"])
        self.assertEqual("raw", images[-1]["format"])
        for box, entry in zip(layout["edge"], images[1:-1]):
            self.assertEqual((box["x"], box["y"]), (entry["x"], entry["y"]))

    def test_climate_covers_the_season(self):
        _, data = open_object("climate/golf.climate.temperate.parkobj")
        self.assertEqual(
            ["march", "april", "may", "june", "july", "august", "september", "october"], list(data["weather"]))
        for month in data["weather"].values():
            # The engine rescales the weights with integer division: they must add up to kWeatherDistSize.
            self.assertEqual(objects.WEATHER_DIST_SIZE, sum(month["distribution"].values()))

    def test_stream_has_one_square_image_per_frame(self):
        _, data = open_object("stream/golf.stream.creek.parkobj")
        self.assertEqual("stream", data["objectType"])
        frames = data["properties"]["image"]["frames"]
        # The icon, then the frames; the frames are raw so the engine reads them as palette indices.
        self.assertEqual(1 + frames, len(data["images"]))
        self.assertTrue(all(i.get("format") == "raw" for i in data["images"][1:]))
        self.assertTrue(data["properties"]["isHazard"])

    def test_building_has_an_icon_and_one_view_per_rotation(self):
        _, data = open_object("building/golf.building.club_house.parkobj")
        self.assertEqual("building", data["objectType"])
        self.assertEqual(1 + 4, len(data["images"]))
        properties = data["properties"]
        self.assertEqual("golf_club", properties["kind"])
        self.assertEqual([3, 2], properties["footprint"])
        self.assertTrue({"food", "drink", "toilet", "pro_shop"} <= set(properties["services"]))
        tile = properties["entrance"]["tile"]
        self.assertTrue(0 <= tile[0] < 3 and 0 <= tile[1] < 2)

    def test_all_building_kinds_and_services(self):
        expected = {
            "club_house": ("golf_club", [3, 2], {"food", "drink", "toilet", "pro_shop"}),
            "food_truck": ("food_truck", [1, 1], {"food", "drink"}),
            "cottage": ("cottage", [2, 2], {"rest"}),
            "driving_range_kiosk": ("driving_range_kiosk", [1, 1], {"pro_shop"}),
            "cart_rental": ("cart_rental", [2, 1], set()),
            "restroom": ("restroom", [1, 1], {"toilet"}),
        }
        for name, (kind, footprint, services) in expected.items():
            _, data = open_object("building/golf.building.%s.parkobj" % name)
            self.assertEqual("building", data["objectType"])
            self.assertEqual(1 + 4, len(data["images"]))
            props = data["properties"]
            self.assertEqual(kind, props["kind"])
            self.assertEqual(footprint, props["footprint"])
            self.assertEqual(services, set(props["services"]))
            tile = props["entrance"]["tile"]
            self.assertTrue(0 <= tile[0] < footprint[0] and 0 <= tile[1] < footprint[1])

    def test_footpath_surfaces_have_every_image_the_paint_code_indexes(self):
        # 51 flat images (edge and corner shapes), then 4 rises x 4 directions of sloped ones, after the icon.
        for name, queue in (("gravel", False), ("tee_queue", True)):
            z, data = open_object("footpath_surface/golf.footpath_surface.%s.parkobj" % name)
            self.assertEqual("footpath_surface", data["objectType"])
            self.assertEqual(1 + 51 + 16, len(data["images"]))
            self.assertEqual(queue, data["properties"]["isQueue"])
            for image in data["images"]:
                self.assertIn(image["path"], z.namelist())

    def test_footpath_railings_cover_the_bridge_images(self):
        z, data = open_object("footpath_railings/golf.footpath_railings.none.parkobj")
        self.assertEqual("footpath_railings", data["objectType"])
        # The bridge images start at 37 after the preview and the paint code reads up to 54 beyond that.
        self.assertGreater(len(data["images"]), 1 + 37 + 54)
        for image in data["images"]:
            self.assertIn(image["path"], z.namelist())

    def test_all_path_addition_kinds_and_properties(self):
        expected = {
            "bench": ("bench", True, False, False, False, False, 1 + 8),
            "bin": ("bin", False, True, False, False, False, 1 + 12),
            "lamp": ("lamp", False, False, True, False, False, 1 + 8),
            "ball_washer": ("ball_washer", False, False, False, False, True, 1 + 8),
            "path_sign": ("sign", False, False, False, True, False, 1 + 8),
        }
        for name, (render_as, is_bench, is_bin, is_lamp, is_sign, is_ball_washer, num_images) in expected.items():
            _, data = open_object("path_addition/golf.path_addition.%s.parkobj" % name)
            self.assertEqual("path_addition", data["objectType"])
            self.assertEqual(num_images, len(data["images"]))
            props = data["properties"]
            self.assertEqual(render_as, props["renderAs"])
            self.assertEqual(is_bench, props.get("isBench", False))
            self.assertEqual(is_bin, props.get("isBin", False))
            self.assertEqual(is_lamp, props.get("isLamp", False))
            self.assertEqual(is_sign, props.get("isSign", False))
            self.assertEqual(is_ball_washer, props.get("isBallWasher", False))
            self.assertGreater(props["price"], 0)

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
