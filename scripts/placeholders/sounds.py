#!/usr/bin/env python3
"""Placeholder sound generator (AST-005).

Generates one short WAV (22050 Hz, 16-bit, mono) per game sound effect of SoundId (src/openrct2/audio/Audio.h) from
tones, sweeps and noise, and packs them as the audio objects the game loads instead of the RCT2/OpenRCT2 sounds:

  golf.audio.base        SoundId values below liftRMC
  golf.audio.additional  liftRMC and later

Each sound has a recognisable shape for its kind (click, chime, buzz, voice, engine hum, water noise, rumble...)
and a pitch derived from its index, so a developer can tell them apart. Output is byte-reproducible on every
platform: the synthesis uses only basic arithmetic (no libm), the noise generator is a fixed LCG, and archives are
stored uncompressed with fixed timestamps.

Usage: sounds.py [--output DIR]   (default: data/object/golf/audio)
"""

import argparse
import json
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from parkobj import ZIP_DATE, pack  # noqa: E402,F401  (ZIP_DATE re-exported for the tests)

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
AUDIO_H = os.path.join(REPO_ROOT, "src", "openrct2", "audio", "Audio.h")
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "data", "object", "golf", "audio")

SAMPLE_RATE = 22050
AMPLITUDE = 0.45
FIRST_ADDITIONAL = "liftRMC"

PI = 3.141592653589793
TWO_PI = 2.0 * PI


def sound_ids(path=AUDIO_H):
    """SoundId enumerator names in value order, up to (excluding) the special values."""
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    body = text[text.index("enum class SoundId : uint8_t\n"):]
    body = body[body.index("{") + 1:body.index("};")]
    names = []
    for line in body.splitlines():
        line = line.split("//")[0].strip().rstrip(",")
        if not line:
            continue
        if "=" in line:
            break  # noScream = 254, null = 255
        names.append(line)
    return names


def sin(x):
    """Deterministic sine (range reduction + Taylor series): only +, -, *, / and exact fmod."""
    x = x % TWO_PI
    if x > PI:
        x -= TWO_PI
    if x > PI / 2:
        x = PI - x
    elif x < -PI / 2:
        x = -PI - x
    x2 = x * x
    term = x
    total = x
    for n in range(1, 8):
        term *= -x2 / ((2 * n) * (2 * n + 1))
        total += term
    return total


class Noise:
    """Deterministic white noise in [-1, 1] (32-bit LCG)."""

    def __init__(self, seed):
        self.state = (seed * 2654435761 + 12345) & 0xFFFFFFFF

    def next(self):
        self.state = (self.state * 1664525 + 1013904223) & 0xFFFFFFFF
        return (self.state >> 8) / float(1 << 23) - 1.0


def samples(duration):
    return int(SAMPLE_RATE * duration)


def envelope(i, n, attack=0.005, release=0.3):
    """Linear attack, exponential-like release towards the end."""
    t = i / SAMPLE_RATE
    a = min(1.0, t / attack) if attack > 0 else 1.0
    remaining = (n - i) / float(n)
    r = min(1.0, remaining / release) if release > 0 else 1.0
    return a * r * r


def tone(freqs, duration, wave="sine", vibrato=0.0, sweep=1.0):
    """Sequence of notes (equal length) of the given waveform; sweep multiplies the pitch over each note."""
    out = []
    per_note = samples(duration / len(freqs))
    for freq in freqs:
        phase = 0.0
        for i in range(per_note):
            k = i / float(per_note)
            f = freq * (1.0 + (sweep - 1.0) * k) * (1.0 + vibrato * sin(TWO_PI * 6.0 * i / SAMPLE_RATE))
            phase += f / SAMPLE_RATE
            p = phase % 1.0
            if wave == "sine":
                v = sin(TWO_PI * p)
            elif wave == "square":
                v = 1.0 if p < 0.5 else -1.0
            else:  # saw
                v = 2.0 * p - 1.0
            out.append(v * envelope(i, per_note, release=0.5))
    return out


def noise(duration, seed, smooth=1, pulses=0, decay=True):
    """Noise smoothed by a moving average of `smooth` samples (darker when larger), optionally pulsed."""
    n = samples(duration)
    gen = Noise(seed)
    window = []
    acc = 0.0
    out = []
    for i in range(n):
        v = gen.next()
        window.append(v)
        acc += v
        if len(window) > smooth:
            acc -= window.pop(0)
        v = max(-1.0, min(1.0, acc / len(window) * (1.0 + smooth ** 0.5 * 0.5)))
        if pulses:
            v *= 1.0 if int(i * pulses * 2 / n) % 2 == 0 else 0.15
        out.append(v * (envelope(i, n, release=0.6) if decay else envelope(i, n, attack=0.05, release=0.08)))
    return out


def mix(*parts):
    n = max(len(p) for p in parts)
    return [sum(p[i] for p in parts if i < len(p)) / len(parts) for i in range(n)]


def synthesise(name, index):
    """The placeholder waveform for one SoundId (floats in [-1, 1])."""
    base = 220.0 + 15.0 * index
    if re.match(r"click\d|placeItem|balloonPop", name):
        return tone([base * 4], 0.05, "square")
    if name == "windowOpen":
        return tone([660.0, 990.0], 0.08)
    if name == "purchase":
        return tone([880.0, 1320.0], 0.25)
    if name == "newsItem":
        return tone([660.0, 880.0, 1100.0], 0.36)
    if name == "error":
        return tone([140.0], 0.22, "square")
    if re.match(r"scream\d|hauntedHouseScream", name):
        return tone([base * 3], 0.6, "saw", vibrato=0.04, sweep=0.6)
    if re.match(r"laugh\d", name):
        return tone([base * 2] * 4, 0.6, "saw", vibrato=0.02, sweep=0.9)
    if re.match(r"cough\d", name):
        return noise(0.3, index, smooth=6, pulses=2)
    if name == "quack":
        return tone([520.0, 470.0], 0.24, "saw", sweep=0.8)
    if name in ("applause", "crowdAmbience"):
        return noise(1.0, index, smooth=2, pulses=12, decay=name == "applause")
    if name == "hauntedHouseScare":
        return mix(tone([90.0], 0.8, "saw", sweep=1.5), noise(0.8, index, smooth=20))
    if re.match(r"thunder\d", name):
        return noise(1.2, index, smooth=60)
    if name == "crash":
        return noise(0.5, index, smooth=3)
    if re.match(r"water\d|waterSplash|layingOutWater|toiletFlush|trackFrictionWater", name):
        return noise(0.6, index, smooth=12)
    if name == "rain":
        return noise(1.0, index, smooth=4, decay=False)
    if re.match(r"rideLaunch\d", name):
        return tone([base], 0.7, "saw", sweep=3.0)
    if name == "trainWhistle":
        return mix(tone([740.0], 0.6), tone([932.0], 0.6))
    if re.match(r"blockBrake|brakeRelease|door|portcullis|mechanicFix", name):
        return mix(tone([base / 2], 0.18), noise(0.18, index, smooth=8))
    if re.match(r"lift|trackFriction|friction|goKartEngine|tram|trainDeparting", name):
        return mix(tone([base / 3], 0.8, "saw", vibrato=0.01), noise(0.8, index, smooth=10, decay=False))
    # Golf sounds (AUD-006)
    if name == "driverSwing":
        return mix(tone([180.0], 0.35, "saw", sweep=2.5), noise(0.35, index, smooth=3))
    if name == "ironSwing":
        return mix(tone([280.0], 0.28, "saw", sweep=2.0), noise(0.28, index, smooth=4))
    if name == "putt":
        return mix(tone([440.0], 0.12, "sine", sweep=0.8), noise(0.12, index, smooth=8))
    if name == "ballImpactGrass":
        return noise(0.15, index, smooth=14)
    if name == "ballImpactSand":
        return noise(0.25, index, smooth=20)
    if name == "ballImpactHard":
        return mix(tone([800.0], 0.08, "square", sweep=0.5), noise(0.08, index, smooth=2))
    if name == "ballImpactWater":
        return noise(0.4, index, smooth=10)
    if name == "ballImpactTree":
        return mix(tone([350.0], 0.18, "saw", sweep=0.7), noise(0.18, index, smooth=6))
    if name == "ballImpactBuilding":
        return mix(tone([600.0], 0.12, "square", sweep=0.6), noise(0.12, index, smooth=3))
    if name == "cupDrop":
        return tone([1200.0, 950.0], 0.2, "sine", sweep=0.9)
    if name == "cartMotorLoop":
        return tone([120.0], 0.6, "saw", vibrato=0.01)
    if name == "carEngineLoop":
        return mix(tone([80.0], 0.8, "saw", vibrato=0.02), noise(0.8, index, smooth=16, decay=False))
    if name == "carDoor":
        return mix(tone([160.0], 0.2, "square"), noise(0.2, index, smooth=6))
    if name == "crowdClap":
        return noise(0.8, index, smooth=2, pulses=8, decay=True)
    if name == "golferCheer":
        return tone([400.0, 500.0, 650.0], 0.5, "saw", vibrato=0.03, sweep=1.2)
    if name == "golferGroan":
        return tone([300.0, 220.0, 160.0], 0.6, "saw", vibrato=0.02, sweep=0.7)
    if name == "golferInsultGrumble":
        return noise(0.5, index, smooth=8, pulses=4)
    if name == "birdChirp":
        return tone([1800.0, 2400.0, 2100.0], 0.25, "sine", vibrato=0.05, sweep=1.3)
    if name == "duck":
        return mix(tone([520.0, 380.0], 0.22, "saw", vibrato=0.04, sweep=0.7), noise(0.22, index, smooth=6))
    if name == "streamLoop":
        return noise(1.0, index, smooth=8, decay=False)
    return tone([base], 0.15)


def wav(values):
    pcm = b"".join(struct.pack("<h", int(round(max(-1.0, min(1.0, v)) * AMPLITUDE * 32767))) for v in values)
    header = struct.pack("<4sI4s", b"RIFF", 36 + len(pcm), b"WAVE")
    fmt = struct.pack("<4sIHHIIHH", b"fmt ", 16, 1, 1, SAMPLE_RATE, SAMPLE_RATE * 2, 2, 16)
    return header + fmt + struct.pack("<4sI", b"data", len(pcm)) + pcm


def snake(name):
    return re.sub(r"(?<=[a-z0-9])([A-Z])", r"_\1", name).lower()


def audio_object(object_id, description, names, first_index):
    sample_paths = []
    files = {}
    for offset, name in enumerate(names):
        path = "samples/%02d_%s.wav" % (first_index + offset, snake(name))
        files[path] = wav(synthesise(name, first_index + offset))
        sample_paths.append(path)
    manifest = {
        "id": object_id,
        "version": "1.0",
        "authors": ["Open Golf Simulator contributors"],
        "licence": "CC0-1.0",
        "sourceGame": ["custom"],
        "objectType": "audio",
        "samples": sample_paths,
        "strings": {"name": {"en-GB": description}},
    }
    files["object.json"] = (json.dumps(manifest, indent=4) + "\n").encode("utf-8")
    return files


def build(path=AUDIO_H):
    """Returns {file name: bytes} of the audio objects."""
    names = sound_ids(path)
    split = names.index(FIRST_ADDITIONAL)
    return {
        "golf.audio.base.parkobj": pack(
            audio_object("golf.audio.base", "Placeholder sound effects (generated).", names[:split], 0)),
        "golf.audio.additional.parkobj": pack(
            audio_object("golf.audio.additional", "Additional placeholder sound effects (generated).", names[split:],
                         split)),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="output directory")
    args = parser.parse_args(argv)
    os.makedirs(args.output, exist_ok=True)
    files = build()
    for name, content in sorted(files.items()):
        with open(os.path.join(args.output, name), "wb") as f:
            f.write(content)
    print("Wrote %d object(s) to %s" % (len(files), args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
