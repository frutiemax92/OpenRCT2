#!/usr/bin/env python3
"""Unit tests for the base graphics pack scripts (AST-002): sprite_ids.py, base_pack_layout.py, base_pack.py."""

import importlib.util
import json
import os
import struct
import sys
import tempfile
import unittest

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
SCRIPTS = os.path.join(ROOT, "scripts", "placeholders")
BASE = os.path.join(ROOT, "resources", "base")


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(SCRIPTS, name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sys.path.insert(0, SCRIPTS)
sprite_ids = load("sprite_ids")
layout_script = load("base_pack_layout")
pack = load("base_pack")
sys.path.remove(SCRIPTS)

VALUES = sprite_ids.load()
with open(os.path.join(BASE, "layout.json"), "r", encoding="utf-8") as f:
    LAYOUT = json.load(f)
FILES = pack.build(LAYOUT)


class SpriteIdsTests(unittest.TestCase):
    def test_parses_every_form(self):
        text = """
enum : ImageIndex
{
    SPR_A = 10,
    SPR_B,
    SPR_C = SPR_A,
    SPR_D = SPR_B + 5, // comment
    SPR_E = SPR_D - 2,
    SPR_F = 0x20,
    SPR_G = SPR_A * 2,
    SPR_H,
    SPR_I = SPR_F + 1
};
"""
        values = sprite_ids.parse(text)
        self.assertEqual({"SPR_A": 10, "SPR_B": 11, "SPR_C": 10, "SPR_D": 16, "SPR_E": 14, "SPR_F": 32, "SPR_I": 33},
                         values)

    def test_real_header_landmarks(self):
        self.assertEqual(29294, VALUES["SPR_G1_END"])
        self.assertEqual(29357, VALUES["SPR_G2_BEGIN"])
        self.assertEqual(VALUES["SPR_G2_BEGIN"] + 1, VALUES["SPR_G2_PLACEHOLDER"])
        self.assertGreater(VALUES["SPR_G2_END"], VALUES["SPR_G2_BEGIN"])


class LayoutScriptTests(unittest.TestCase):
    def test_frames_follow_referenced_names(self):
        values = {"SPR_A": 5, "SPR_B": 8, "SPR_FAR": 100, "SPR_CHAR_START": 200, "SPR_G1_END": 300}
        slots = layout_script.g1_slots(values, {"SPR_A", "SPR_FAR", "SPR_CHAR_START", "SPR_UNKNOWN"})
        self.assertEqual({5: ("SPR_A", 0), 6: ("SPR_A", 1), 7: ("SPR_A", 2), 100: ("SPR_FAR", 0)}, slots)

    def test_reads_only_the_element_table(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "elements.bin")
            with open(path, "wb") as f:
                f.write(struct.pack("<II", 2, 0))
                f.write(layout_script.STORED_ELEMENT.pack(0, 12, 7, -1, 2, 0, 0))
                f.write(layout_script.STORED_ELEMENT.pack(0, 3, 4, 0, 0, 8, 0))
            elements = layout_script.read_g1_elements(path, 2)
            self.assertEqual((0, 12, 7, -1, 2, 0, 0), elements[0])
            with self.assertRaises(RuntimeError):
                layout_script.read_g1_elements(path, 3)

    def test_png_size(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "a.png")
            with open(path, "wb") as f:
                f.write(load("pixelart").Image(7, 3).png())
            self.assertEqual((7, 3), layout_script.png_size(path))


class LayoutFileTests(unittest.TestCase):
    def test_slots_increase_and_stay_in_range(self):
        slots = [e["slot"] for e in LAYOUT]
        self.assertEqual(sorted(set(slots)), slots)
        self.assertGreaterEqual(slots[0], 0)
        self.assertLess(slots[-1], VALUES["SPR_G2_END"])

    def test_names_match_slots(self):
        for entry in LAYOUT:
            if entry["name"].startswith("SPR_G2_SLOT_"):
                index = int(entry["name"][len("SPR_G2_SLOT_"):])
                self.assertEqual(VALUES["SPR_G2_BEGIN"] + index, entry["slot"])
            else:
                self.assertEqual(VALUES[entry["name"]] + entry["frame"], entry["slot"], entry["name"])

    def test_every_g2_slot_is_present(self):
        g2 = [e for e in LAYOUT if VALUES["SPR_G2_BEGIN"] <= e["slot"] < VALUES["SPR_G2_END"]]
        self.assertEqual(VALUES["SPR_G2_END"] - VALUES["SPR_G2_BEGIN"], len(g2))

    def test_key_ui_sprites_are_present(self):
        names = {e["name"] for e in LAYOUT if e["frame"] == 0}
        for name in ("SPR_RESIZE", "SPR_MENU_NEW_GAME", "SPR_MENU_LOAD_GAME", "SPR_TAB", "SPR_G2_LOGO"):
            self.assertIn(name, names)

    def test_no_sprite_font_glyphs(self):
        self.assertFalse([e for e in LAYOUT if e["name"].startswith(("SPR_CHAR", "SPR_FONTS"))])

    def test_sizes_are_positive(self):
        for entry in LAYOUT:
            self.assertGreater(entry["width"], 0, entry)
            self.assertGreater(entry["height"], 0, entry)


class PackTests(unittest.TestCase):
    def test_committed_pack_is_up_to_date(self):
        """resources/base must be regenerated (scripts/placeholders/base_pack.py) after a change."""
        for path, content in FILES.items():
            with open(os.path.join(BASE, path), "rb") as f:
                self.assertEqual(content, f.read(), "%s is stale: rerun scripts/placeholders/base_pack.py" % path)
        committed = set()
        for dirpath, _, filenames in os.walk(os.path.join(BASE, "images")):
            for name in filenames:
                committed.add(os.path.relpath(os.path.join(dirpath, name), BASE).replace(os.sep, "/"))
        self.assertEqual({p for p in FILES if p.startswith("images/")}, committed)

    def test_sprites_json_matches_layout(self):
        sprites = json.loads(FILES["sprites.json"])
        self.assertEqual([e["slot"] for e in LAYOUT], [s["slot"] for s in sprites])
        for entry, sprite in zip(LAYOUT, sprites):
            self.assertEqual((entry["x"], entry["y"]), (sprite["x"], sprite["y"]))
            png = FILES[sprite["path"]]
            self.assertEqual((entry["width"], entry["height"]), struct.unpack(">II", png[16:24]))

    def test_decreasing_layout_is_rejected(self):
        entry = {"slot": 5, "name": "SPR_A", "frame": 0, "width": 4, "height": 4, "x": 0, "y": 0}
        with self.assertRaises(ValueError):
            pack.build([entry, dict(entry)])

    def test_labels_fit(self):
        self.assertEqual("TAB PARK", pack.label_for("SPR_TAB_PARK", 0, 40, 12))
        self.assertEqual("TP", pack.label_for("SPR_TAB_PARK", 0, 12, 12))
        self.assertEqual("TP 3", pack.label_for("SPR_TAB_PARK", 3, 18, 12))
        self.assertEqual("", pack.label_for("SPR_TAB_PARK", 0, 40, 6))
        self.assertEqual("", pack.label_for("SPR_TAB_PARK", 0, 4, 12))

    def test_frames_share_a_hue(self):
        a = pack.colour_for("SPR_TAB_PARK", 0)
        b = pack.colour_for("SPR_TAB_PARK", 2)
        self.assertNotEqual(a, b)
        self.assertEqual(a, pack.colour_for("SPR_TAB_PARK", 0))


if __name__ == "__main__":
    unittest.main()
