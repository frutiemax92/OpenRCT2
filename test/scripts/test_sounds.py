#!/usr/bin/env python3
"""Unit tests for scripts/placeholders/sounds.py (AST-005)."""

import importlib.util
import io
import json
import math
import os
import tempfile
import unittest
import wave
import zipfile

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
spec = importlib.util.spec_from_file_location("sounds", os.path.join(ROOT, "scripts", "placeholders", "sounds.py"))
sounds = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sounds)

NAMES = sounds.sound_ids()
FILES = sounds.build()
COMMITTED = os.path.join(ROOT, "data", "object", "golf", "audio")


def open_object(name):
    z = zipfile.ZipFile(io.BytesIO(FILES[name]))
    return z, json.loads(z.read("object.json"))


class SoundIdTests(unittest.TestCase):
    def test_parses_the_enum(self):
        self.assertEqual("liftClassic", NAMES[0])
        self.assertIn("portcullis", NAMES)
        self.assertEqual("liftFlume", NAMES[-1])
        self.assertNotIn("noScream", NAMES)
        self.assertNotIn("null", NAMES)
        self.assertEqual(len(NAMES), len(set(NAMES)))

    def test_parse_stops_at_special_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "Audio.h")
            with open(path, "w") as f:
                f.write("enum class SoundId : uint8_t\n{\n    a,\n    b, // comment\n\n    c,\n    noScream = 254,\n    d\n};\n")
            self.assertEqual(["a", "b", "c"], sounds.sound_ids(path))


class SynthesisTests(unittest.TestCase):
    def test_sine_matches_libm(self):
        for k in range(-4000, 4000):
            x = k / 97.0
            self.assertAlmostEqual(math.sin(x), sounds.sin(x), places=9)

    def test_noise_is_deterministic_and_bounded(self):
        a = [sounds.Noise(7).next() for _ in range(1)]
        n1, n2 = sounds.Noise(7), sounds.Noise(7)
        values = [n1.next() for _ in range(5000)]
        self.assertEqual(values, [n2.next() for _ in range(5000)])
        self.assertTrue(all(-1.0 <= v <= 1.0 for v in values))
        self.assertNotEqual(a, [sounds.Noise(8).next()])

    def test_every_sound_is_audible_and_distinct(self):
        waves = {}
        for index, name in enumerate(NAMES):
            values = sounds.synthesise(name, index)
            self.assertGreater(len(values), sounds.SAMPLE_RATE // 50, name)
            self.assertGreater(max(abs(v) for v in values), 0.05, name)
            self.assertTrue(all(-1.0 <= v <= 1.0 for v in values), name)
            waves[name] = tuple(round(v, 6) for v in values)
        self.assertEqual(len(NAMES), len(set(waves.values())))

    def test_wav_format(self):
        data = sounds.wav([0.0, 1.0, -1.0, 2.0])
        with wave.open(io.BytesIO(data)) as w:
            self.assertEqual((1, 2, sounds.SAMPLE_RATE, 4), (w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()))

    def test_snake_case(self):
        self.assertEqual("lift_rmc", sounds.snake("liftRMC"))
        self.assertEqual("click1", sounds.snake("click1"))
        self.assertEqual("haunted_house_scream2", sounds.snake("hauntedHouseScream2"))


class ObjectTests(unittest.TestCase):
    def test_build_is_deterministic(self):
        self.assertEqual(FILES, sounds.build())

    def test_committed_objects_are_up_to_date(self):
        """data/object/golf/audio must be regenerated (scripts/placeholders/sounds.py) after a change."""
        self.assertEqual(set(FILES), set(os.listdir(COMMITTED)))
        for name, content in FILES.items():
            with open(os.path.join(COMMITTED, name), "rb") as f:
                self.assertEqual(content, f.read(), "%s is stale: rerun scripts/placeholders/sounds.py" % name)

    def test_samples_follow_sound_ids(self):
        split = NAMES.index("liftRMC")
        expected = {"golf.audio.base.parkobj": NAMES[:split], "golf.audio.additional.parkobj": NAMES[split:]}
        for name, ids in expected.items():
            z, manifest = open_object(name)
            self.assertEqual("audio", manifest["objectType"])
            self.assertEqual(name[:-len(".parkobj")], manifest["id"])
            self.assertEqual("CC0-1.0", manifest["licence"])
            self.assertEqual(len(ids), len(manifest["samples"]))
            for path, sound in zip(manifest["samples"], ids):
                self.assertTrue(path.endswith(sounds.snake(sound) + ".wav"), (path, sound))
                with wave.open(io.BytesIO(z.read(path))) as w:
                    self.assertEqual(1, w.getnchannels())
                    self.assertEqual(2, w.getsampwidth())
                    self.assertGreater(w.getnframes(), 0)

    def test_archives_are_stored_with_fixed_metadata(self):
        for name in FILES:
            z, _ = open_object(name)
            infos = z.infolist()
            self.assertEqual("object.json", infos[0].filename)
            for info in infos:
                self.assertEqual(zipfile.ZIP_STORED, info.compress_type)
                self.assertEqual(sounds.ZIP_DATE, info.date_time)


if __name__ == "__main__":
    unittest.main()
