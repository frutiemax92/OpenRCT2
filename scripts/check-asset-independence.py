#!/usr/bin/env python3
"""Asset independence check (AST-006).

Fails when:
  1. a source file references RCT/OpenRCT2 graphics archives (g1.dat, g2.dat, csg1*.dat) more often than
     recorded in the baseline. The baseline lists the references that still exist while the engine is migrated
     to the base asset pack (AST-002); it may only shrink. Run with --update-baseline after removing references.
  2. a file under data/ (including files inside .parkobj/.parkap/.zip archives) is byte-identical to an
     OpenRCT2 media asset of the upstream reference tree, or to a file of a local RCT installation passed with
     --known-dir.
  3. a file under data/ uses an RCT file format (.dat, .sv4, .sv6, .sc4, .sc6, .td4, .td6).

Exit status: 0 = clean, 1 = violations found, 2 = usage or environment error.
"""

import argparse
import hashlib
import os
import re
import subprocess
import sys
import zipfile

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# Last upstream OpenRCT2 commit the fork was created from. Its media files are the "known OpenRCT2 assets".
UPSTREAM_REF = "53bbc400e28be44d3c2e56fa514d5f65b61a64d4"

BASELINE_PATH = os.path.join("scripts", "asset-independence-baseline.txt")

REFERENCE_PATTERN = re.compile(r"\b(?:g1|g2)\.dat\b|\bcsg1[a-z]*\.dat\b", re.IGNORECASE)

# Where code references are searched (relative to the repository root).
REFERENCE_ROOTS = ["src", "test", "cmake", "data", "CMakeLists.txt"]
REFERENCE_EXCLUDED_DIRS = {os.path.join("src", "thirdparty")}
# The tests of this check necessarily contain the names it looks for.
REFERENCE_EXCLUDED_FILES = {"test/scripts/test_check_asset_independence.py"}
REFERENCE_EXTENSIONS = {".c", ".cpp", ".h", ".hpp", ".mm", ".cmake", ".txt", ".json", ".js", ".ts", ".py", ".in"}

MEDIA_EXTENSIONS = {
    ".png", ".bmp", ".gif", ".jpg", ".jpeg", ".wav", ".ogg", ".flac", ".mp3", ".mid",
    ".ttf", ".otf", ".dat", ".parkobj", ".parkap", ".sv4", ".sv6", ".sc4", ".sc6", ".td4", ".td6",
}
FORBIDDEN_EXTENSIONS = {".dat", ".sv4", ".sv6", ".sc4", ".sc6", ".td4", ".td6"}
ARCHIVE_EXTENSIONS = {".parkobj", ".parkap", ".zip"}


def git_blob_id(content):
    """Same id git gives a blob, so upstream assets can be fingerprinted without extracting them."""
    header = b"blob %d\0" % len(content)
    return hashlib.sha1(header + content).hexdigest()


def extension(path):
    return os.path.splitext(path)[1].lower()


def relpath(path, root):
    return os.path.relpath(path, root).replace(os.sep, "/")


# ---------------------------------------------------------------------------------------------------------------
# 1. Code references

def iter_reference_files(root):
    for entry in REFERENCE_ROOTS:
        full = os.path.join(root, entry)
        if os.path.isfile(full):
            yield full
            continue
        for dirpath, dirnames, filenames in os.walk(full):
            rel_dir = os.path.relpath(dirpath, root)
            dirnames[:] = sorted(d for d in dirnames if os.path.join(rel_dir, d) not in REFERENCE_EXCLUDED_DIRS)
            for name in sorted(filenames):
                if extension(name) in REFERENCE_EXTENSIONS:
                    yield os.path.join(dirpath, name)


def count_references(root):
    counts = {}
    for path in iter_reference_files(root):
        if relpath(path, root) in REFERENCE_EXCLUDED_FILES:
            continue
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as f:
                n = len(REFERENCE_PATTERN.findall(f.read()))
        except OSError:
            continue
        if n:
            counts[relpath(path, root)] = n
    return counts


def read_baseline(path):
    baseline = {}
    if not os.path.exists(path):
        return baseline
    with open(path, "r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.rsplit(None, 1)
            if len(parts) != 2 or not parts[1].isdigit():
                raise ValueError("%s:%d: expected '<path> <count>'" % (path, lineno))
            baseline[parts[0]] = int(parts[1])
    return baseline


def write_baseline(path, counts):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("# References to g1.dat / g2.dat / csg1*.dat that remain while the engine moves to the base asset\n")
        f.write("# pack (AST-002, AST-006). This list may only shrink: regenerate it with\n")
        f.write("#   scripts/check-asset-independence.py --update-baseline\n")
        f.write("# after removing references. The goal is an empty list.\n")
        for name in sorted(counts):
            f.write("%s %d\n" % (name, counts[name]))


def compare_references(counts, baseline):
    """Returns (violations, improvements) as lists of messages."""
    violations = []
    improvements = []
    for name in sorted(set(counts) | set(baseline)):
        now = counts.get(name, 0)
        allowed = baseline.get(name, 0)
        if now > allowed:
            violations.append("%s: %d reference(s) to RCT/OpenRCT2 graphics archives, baseline allows %d" % (name, now, allowed))
        elif now < allowed:
            improvements.append("%s: %d reference(s), baseline still lists %d" % (name, now, allowed))
    return violations, improvements


# ---------------------------------------------------------------------------------------------------------------
# 2. Known assets

def upstream_media_blobs(root, ref):
    """Maps git blob id -> upstream path for every media file of the upstream reference tree."""
    try:
        out = subprocess.run(
            ["git", "-C", root, "ls-tree", "-r", ref], check=True, capture_output=True, text=True
        ).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        raise RuntimeError("cannot list upstream tree %s (fetch full history?): %s" % (ref, e))
    blobs = {}
    for line in out.splitlines():
        meta, _, path = line.partition("\t")
        fields = meta.split()
        if len(fields) == 3 and fields[1] == "blob" and extension(path) in MEDIA_EXTENSIONS:
            blobs.setdefault(fields[2], "%s@upstream" % path)
    return blobs


def known_dir_blobs(directory):
    blobs = {}
    for dirpath, _, filenames in os.walk(directory):
        for name in sorted(filenames):
            full = os.path.join(dirpath, name)
            try:
                with open(full, "rb") as f:
                    blobs.setdefault(git_blob_id(f.read()), full)
            except OSError:
                continue
    return blobs


def iter_data_files(data_dir):
    """Yields (display name, content) for every file under data_dir, including archive members."""
    for dirpath, dirnames, filenames in os.walk(data_dir):
        dirnames.sort()
        for name in sorted(filenames):
            full = os.path.join(dirpath, name)
            try:
                with open(full, "rb") as f:
                    content = f.read()
            except OSError:
                continue
            display = relpath(full, os.path.dirname(data_dir))
            yield display, content
            if extension(name) in ARCHIVE_EXTENSIONS:
                yield from iter_archive_members(full, display)


def iter_archive_members(path, display):
    try:
        with zipfile.ZipFile(path) as z:
            for info in z.infolist():
                if not info.is_dir():
                    yield "%s!%s" % (display, info.filename), z.read(info)
    except (zipfile.BadZipFile, OSError):
        return


def check_data(data_dir, known):
    violations = []
    if not os.path.isdir(data_dir):
        return violations
    for display, content in iter_data_files(data_dir):
        member = display.rsplit("!", 1)[-1]
        if extension(member) in FORBIDDEN_EXTENSIONS:
            violations.append("%s: RCT file format is not allowed" % display)
        source = known.get(git_blob_id(content))
        if source is not None:
            violations.append("%s: byte-identical to known asset %s" % (display, source))
    return violations


# ---------------------------------------------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default=REPO_ROOT, help="repository root (default: this checkout)")
    parser.add_argument("--baseline", help="baseline file (default: %s under the root)" % BASELINE_PATH)
    parser.add_argument("--upstream-ref", default=UPSTREAM_REF, help="upstream commit whose media are known assets")
    parser.add_argument("--no-upstream", action="store_true", help="skip the upstream asset fingerprints")
    parser.add_argument("--known-dir", action="append", default=[],
                        help="directory of known assets, e.g. an RCT2 installation (repeatable)")
    parser.add_argument("--update-baseline", action="store_true", help="rewrite the baseline from the current code")
    args = parser.parse_args(argv)

    root = os.path.abspath(args.root)
    baseline_path = args.baseline or os.path.join(root, BASELINE_PATH)

    counts = count_references(root)
    if args.update_baseline:
        write_baseline(baseline_path, counts)
        print("Baseline written: %d file(s), %d reference(s)" % (len(counts), sum(counts.values())))
        return 0

    try:
        baseline = read_baseline(baseline_path)
        known = {}
        if not args.no_upstream:
            known.update(upstream_media_blobs(root, args.upstream_ref))
        for directory in args.known_dir:
            if not os.path.isdir(directory):
                raise RuntimeError("--known-dir %s is not a directory" % directory)
            known.update(known_dir_blobs(directory))
    except (RuntimeError, ValueError) as e:
        print("error: %s" % e, file=sys.stderr)
        return 2

    violations, improvements = compare_references(counts, baseline)
    violations += check_data(os.path.join(root, "data"), known)

    for message in improvements:
        print("note: %s (run --update-baseline)" % message)
    for message in violations:
        print("error: %s" % message)
    if violations:
        print("Asset independence check failed: %d violation(s)" % len(violations))
        return 1
    print("Asset independence check passed (%d known asset(s), %d baseline reference(s))"
          % (len(known), sum(counts.values())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
