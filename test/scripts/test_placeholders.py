#!/usr/bin/env python3
"""Unit tests for scripts/placeholders/generate.py (AST-004)."""

import importlib.util
import json
import os
import struct
import tempfile
import unittest
import zlib

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
SCRIPT = os.path.join(ROOT, "scripts", "placeholders", "generate.py")
COMMITTED = os.path.join(ROOT, "resources", "placeholders")

spec = importlib.util.spec_from_file_location("placeholders_generate", SCRIPT)
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

FILES = gen.build()
MANIFEST = json.loads(FILES["manifest.json"])


def decode_png(data):
    """Minimal RGBA8 PNG decoder (filter type 0 only, as written by the generator). Validates every CRC."""
    assert data[:8] == b"\x89PNG\r\n\x1a\n", "bad signature"
    pos = 8
    chunks = []
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        kind = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + length]
        (crc,) = struct.unpack(">I", data[pos + 8 + length:pos + 12 + length])
        assert crc == zlib.crc32(kind + body) & 0xFFFFFFFF, "bad CRC in %r" % kind
        chunks.append((kind, body))
        pos += 12 + length
    assert [k for k, _ in chunks] == [b"IHDR", b"IDAT", b"IEND"]
    width, height, depth, colour_type, _, _, interlace = struct.unpack(">IIBBBBB", chunks[0][1])
    assert (depth, colour_type, interlace) == (8, 6, 0)
    raw = zlib.decompress(chunks[1][1])
    stride = 1 + width * 4
    assert len(raw) == stride * height
    rows = []
    for y in range(height):
        row = raw[y * stride:(y + 1) * stride]
        assert row[0] == 0, "unexpected filter"
        rows.append([tuple(row[1 + x * 4:5 + x * 4]) for x in range(width)])
    return width, height, rows


class ReproducibilityTests(unittest.TestCase):
    def test_build_is_deterministic(self):
        self.assertEqual(FILES, gen.build())

    def test_committed_files_are_up_to_date(self):
        """resources/placeholders must be regenerated (scripts/placeholders/generate.py) after a change."""
        for path, content in FILES.items():
            with open(os.path.join(COMMITTED, path), "rb") as f:
                self.assertEqual(content, f.read(), "%s is stale: rerun scripts/placeholders/generate.py" % path)
        committed = set()
        for dirpath, _, filenames in os.walk(COMMITTED):
            for name in filenames:
                committed.add(os.path.relpath(os.path.join(dirpath, name), COMMITTED).replace(os.sep, "/"))
        self.assertEqual(set(FILES), committed, "unexpected or missing files in resources/placeholders")

    def test_main_writes_every_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(0, gen.main(["--output", tmp]))
            for path, content in FILES.items():
                with open(os.path.join(tmp, path), "rb") as f:
                    self.assertEqual(content, f.read())


class ImageTests(unittest.TestCase):
    def test_every_image_is_a_valid_png_matching_the_manifest(self):
        listed = {entry["path"]: entry for entry in MANIFEST["images"]}
        self.assertEqual(set(listed), {p for p in FILES if p.endswith(".png")})
        for path, entry in listed.items():
            width, height, _ = decode_png(FILES[path])
            self.assertEqual(entry["size"], [width, height], path)

    def test_initial_surface_set(self):
        # TER-042 table, the source spec's list (§6.5) and MPT-001.
        expected = {"green", "tee", "fairway", "grass", "rough", "deep_rough", "sand", "woods", "rocks", "mini_putt_carpet"}
        names = {e["path"][len("terrain/"):-len(".png")] for e in MANIFEST["images"] if e["kind"] == "terrain_surface"}
        self.assertEqual(expected, names)
        for name in expected:
            self.assertIn("terrain/%s_icon.png" % name, FILES)

    def test_terrain_textures_are_square_powers_of_two(self):
        for entry in MANIFEST["images"]:
            if entry["kind"] in ("terrain_surface", "terrain_edge"):
                w, h = entry["size"]
                self.assertEqual(w, h, entry["path"])
                self.assertEqual(0, w & (w - 1), entry["path"])

    def test_placeholders_are_distinct(self):
        for kind in ("terrain_surface", "terrain_surface_icon", "debug_slope_level"):
            entries = [e for e in MANIFEST["images"] if e["kind"] == kind]
            self.assertEqual(len(entries), len({e["colour"] for e in entries}), "colours of %s" % kind)
            self.assertEqual(len(entries), len({e["label"] for e in entries}), "labels of %s" % kind)
            self.assertEqual(len(entries), len({FILES[e["path"]] for e in entries}), "images of %s" % kind)

    def test_surface_textures_tile_seamlessly(self):
        """The checker continues across the wrap: opposite edge pixels belong to neighbouring cells."""
        self.assertEqual(0, gen.TEXTURE_SIZE % (2 * gen.CHECKER))
        for entry in MANIFEST["images"]:
            if entry["kind"] != "terrain_surface":
                continue
            _, _, rows = decode_png(FILES[entry["path"]])
            y = gen.TEXTURE_SIZE - 4  # below the label, away from the origin mark
            self.assertNotEqual(rows[y][0], rows[y][gen.TEXTURE_SIZE - 1], entry["path"])
            self.assertEqual(rows[y][0], rows[y][2 * gen.CHECKER], entry["path"])

    def test_every_image_shows_its_origin(self):
        for entry in MANIFEST["images"]:
            if entry["kind"] in ("terrain_surface", "terrain_edge", "debug_quadrants", "debug_slope_level"):
                _, _, rows = decode_png(FILES[entry["path"]])
                self.assertEqual(rows[0][0], rows[2][2], entry["path"])
                self.assertNotEqual(rows[0][0], rows[0][3], entry["path"])

    def test_labels_use_known_glyphs(self):
        for _, label, _, _ in gen.SURFACES + gen.EDGES:
            for ch in label:
                self.assertTrue(ch in gen.FONT or ch in gen.WIDE_FONT, ch)
        for rows in gen.WIDE_FONT.values():
            self.assertEqual(gen.GLYPH_H, len(rows))
            self.assertEqual(1, len({len(r) for r in rows}))

    def test_text_size(self):
        self.assertEqual((11, 5), gen.text_size("ABC", 1))
        self.assertEqual((26, 10), gen.text_size("FWY", 2))


class QuadrantTests(unittest.TestCase):
    def test_quadrant_of_edge_midpoints(self):
        s = 64
        self.assertEqual(0, gen.quadrant_of(0, 32, s))   # -x edge
        self.assertEqual(1, gen.quadrant_of(32, 63, s))  # +y edge
        self.assertEqual(2, gen.quadrant_of(63, 32, s))  # +x edge
        self.assertEqual(3, gen.quadrant_of(32, 0, s))   # -y edge

    def test_quadrants_have_equal_area(self):
        s = 64
        counts = [0, 0, 0, 0]
        for y in range(s):
            for x in range(s):
                counts[gen.quadrant_of(x, y, s)] += 1
        self.assertEqual(s * s, sum(counts))
        self.assertEqual(counts[0], counts[2])
        self.assertEqual(counts[1], counts[3])
        self.assertLessEqual(abs(counts[0] - counts[1]), 2 * s)

    def test_quadrant_texture_matches_reference_colours(self):
        _, _, rows = decode_png(FILES["debug/quadrants.png"])
        colours = {label: colour + (255,) for label, colour in gen.QUADRANTS}
        self.assertEqual(colours["-X"], rows[40][3])
        self.assertEqual(colours["+Y"], rows[61][40])
        self.assertEqual(colours["+X"], rows[40][61])
        self.assertEqual(colours["-Y"], rows[3][40])


if __name__ == "__main__":
    unittest.main()
