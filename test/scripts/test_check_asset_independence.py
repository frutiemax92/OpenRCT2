#!/usr/bin/env python3
"""Unit tests for scripts/check-asset-independence.py (AST-006)."""

import contextlib
import importlib.util
import io
import os
import subprocess
import tempfile
import unittest
import zipfile

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts", "check-asset-independence.py")

spec = importlib.util.spec_from_file_location("check_asset_independence", SCRIPT)
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def write(root, rel, content):
    path = os.path.join(root, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(content if isinstance(content, bytes) else content.encode("utf-8"))
    return path


def run_main(*args):
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        code = check.main(list(args))
    return code, out.getvalue()


class BlobIdTests(unittest.TestCase):
    def test_matches_git_hash_object(self):
        with tempfile.TemporaryDirectory() as tmp:
            content = b"\x89PNG\r\n\x1a\nnot really a png"
            path = write(tmp, "a.png", content)
            expected = subprocess.run(["git", "hash-object", path], check=True, capture_output=True, text=True).stdout.strip()
            self.assertEqual(expected, check.git_blob_id(content))

    def test_empty_content(self):
        self.assertEqual("e69de29bb2d1d6434b8b29ae775ad8c2e48c5391", check.git_blob_id(b""))


class ReferenceTests(unittest.TestCase):
    def test_counts_every_archive_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "src/a.cpp", 'Load("g1.dat"); Load("G2.DAT"); Load("csg1i.dat"); Load("csg1.dat");')
            write(tmp, "src/b.h", "// no reference, g1.data and xg1.dat are different names")
            write(tmp, "CMakeLists.txt", "install(g2.dat)")
            self.assertEqual({"src/a.cpp": 4, "CMakeLists.txt": 1}, check.count_references(tmp))

    def test_ignores_thirdparty_and_other_extensions(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "src/thirdparty/lib.cpp", "g1.dat")
            write(tmp, "src/readme.md", "g1.dat")
            write(tmp, "docs/a.cpp", "g1.dat")
            self.assertEqual({}, check.count_references(tmp))

    def test_compare_against_baseline(self):
        violations, improvements = check.compare_references(
            {"same": 2, "grew": 3, "new": 1, "shrank": 1}, {"same": 2, "grew": 2, "shrank": 4, "gone": 1}
        )
        self.assertEqual(2, len(violations))
        self.assertTrue(any(v.startswith("grew:") for v in violations))
        self.assertTrue(any(v.startswith("new:") for v in violations))
        self.assertEqual(2, len(improvements))
        self.assertTrue(any(i.startswith("shrank:") for i in improvements))
        self.assertTrue(any(i.startswith("gone:") for i in improvements))

    def test_baseline_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "baseline.txt")
            counts = {"src/x.cpp": 3, "CMakeLists.txt": 1}
            check.write_baseline(path, counts)
            self.assertEqual(counts, check.read_baseline(path))

    def test_missing_baseline_is_empty(self):
        self.assertEqual({}, check.read_baseline("/nonexistent/baseline.txt"))

    def test_malformed_baseline_is_an_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write(tmp, "baseline.txt", "src/x.cpp three\n")
            with self.assertRaises(ValueError):
                check.read_baseline(path)


class DataTests(unittest.TestCase):
    def test_clean_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "data/images/fairway.png", b"our own art")
            self.assertEqual([], check.check_data(os.path.join(tmp, "data"), {}))

    def test_byte_identical_file_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "data/copied.png", b"openrct2 art")
            known = {check.git_blob_id(b"openrct2 art"): "resources/g2/icon.png@upstream"}
            violations = check.check_data(os.path.join(tmp, "data"), known)
            self.assertEqual(1, len(violations))
            self.assertIn("data/copied.png", violations[0])
            self.assertIn("resources/g2/icon.png@upstream", violations[0])

    def test_archive_members_are_checked(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "data", "object", "golf.terrain.sand.parkobj")
            os.makedirs(os.path.dirname(path))
            with zipfile.ZipFile(path, "w") as z:
                z.writestr("object.json", "{}")
                z.writestr("images/sand.png", b"rct sand")
            known = {check.git_blob_id(b"rct sand"): "/rct2/Data/g1.dat"}
            violations = check.check_data(os.path.join(tmp, "data"), known)
            self.assertEqual(1, len(violations))
            self.assertIn("golf.terrain.sand.parkobj!images/sand.png", violations[0])

    def test_rct_formats_are_forbidden(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "data/scenario/park.SV6", b"x")
            write(tmp, "data/object/ride.DAT", b"y")
            path = os.path.join(tmp, "data", "pack.zip")
            with zipfile.ZipFile(path, "w") as z:
                z.writestr("track.td6", b"z")
            violations = check.check_data(os.path.join(tmp, "data"), {})
            self.assertEqual(3, len(violations))
            self.assertTrue(all("RCT file format" in v for v in violations))

    def test_corrupt_archive_does_not_crash(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "data/broken.parkobj", b"not a zip")
            self.assertEqual([], check.check_data(os.path.join(tmp, "data"), {}))

    def test_missing_data_dir(self):
        self.assertEqual([], check.check_data("/nonexistent/data", {}))

    def test_known_dir_fingerprints(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "rct2/Data/csg1.dat", b"original")
            blobs = check.known_dir_blobs(os.path.join(tmp, "rct2"))
            self.assertIn(check.git_blob_id(b"original"), blobs)


class MainTests(unittest.TestCase):
    def test_exit_codes(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "src/a.cpp", "g1.dat")
            code, out = run_main("--root", tmp, "--no-upstream")
            self.assertEqual(1, code, out)

            code, out = run_main("--root", tmp, "--update-baseline")
            self.assertEqual(0, code, out)
            code, out = run_main("--root", tmp, "--no-upstream")
            self.assertEqual(0, code, out)

            write(tmp, "src/a.cpp", "")
            code, out = run_main("--root", tmp, "--no-upstream")
            self.assertEqual(0, code, out)
            self.assertIn("run --update-baseline", out)

    def test_known_dir_violation(self):
        with tempfile.TemporaryDirectory() as tmp:
            write(tmp, "rct/sound.wav", b"original sound")
            write(tmp, "repo/data/sound/swing.wav", b"original sound")
            code, out = run_main("--root", os.path.join(tmp, "repo"), "--no-upstream", "--known-dir", os.path.join(tmp, "rct"))
            self.assertEqual(1, code, out)
            self.assertIn("byte-identical", out)

    def test_bad_known_dir_is_usage_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, out = run_main("--root", tmp, "--no-upstream", "--known-dir", os.path.join(tmp, "missing"))
            self.assertEqual(2, code, out)

    def test_bad_upstream_ref_is_usage_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, out = run_main("--root", tmp, "--upstream-ref", "0" * 40)
            self.assertEqual(2, code, out)

    def test_repository_is_clean(self):
        """The checked-in baseline matches this checkout (the CI gate)."""
        code, out = run_main()
        self.assertEqual(0, code, out)


if __name__ == "__main__":
    unittest.main()
