"""Resolve species names and generate classifier policy from ordered model labels."""

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import unicodedata

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODELS = ROOT / "models/western-us-v2"
WORLDWIDE = ROOT / "models/worldwide-v2"
HUMAN_VOCAL = "Human vocal_Human vocal"
SOUNDS = frozenset(
    f"{sound}_{sound}" for sound in (
        "Engine", "Environmental", "Fireworks", "Gun", "Human non-vocal",
        "Human vocal", "Human whistle", "Noise", "Power tools", "Siren",
    )
)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_pinned(path):
    pairs = (line.split(maxsplit=1) for line in (ROOT / "models/SHA256SUMS").read_text().splitlines())
    checksums = {name: checksum for checksum, name in pairs}
    data = path.read_bytes()
    if sha256(data) != checksums[path.relative_to(ROOT / "models").as_posix()]:
        raise ValueError(f"Model asset differs from the pinned checksum: {path.name}")
    return data


def load_labels(data):
    labels = data.decode("utf-8").splitlines()
    if (not labels or len(set(labels)) != len(labels)
            or any("_" not in label or "\0" in label for label in labels)):
        raise ValueError("Expected unique, nonempty labels in Scientific name_Common name format")
    return labels


def resolve_species(lines, labels):
    def normalize(text):
        return unicodedata.normalize("NFC", " ".join(text.split())).casefold()

    names = defaultdict(set)
    exact = {}
    for index, label in enumerate(labels):
        scientific, common = label.split("_", 1)
        exact[normalize(label)] = index
        for name in (scientific, common):
            names[normalize(name)].add(index)

    selected = set()
    problems = []
    for line, text in enumerate(lines, 1):
        name = normalize(text)
        if not name or name.startswith("#"):
            continue
        if name in exact:
            selected.add(exact[name])
            continue
        matches = sorted(names.get(name, ()))
        if len(matches) == 1:
            selected.add(matches[0])
        elif matches:
            choices = " or ".join(labels[index] for index in matches)
            problems.append(f"line {line}: {text.strip()!r} is ambiguous. Use {choices}.")
        else:
            problems.append(f"line {line}: {text.strip()!r} is absent from the selected model's labels.")
    if problems:
        raise ValueError("\n".join(problems))
    if not selected:
        raise ValueError("Add at least one species to the list")
    return sorted(selected)


def bitset(labels, members):
    flags = bytearray((len(labels) + 7) // 8)
    for index, label in enumerate(labels):
        if label in members:
            flags[index // 8] |= 1 << (index % 8)
    return ", ".join(f"0x{value:02x}" for value in flags)


def render_profile(labels, guard_only, deployment):
    if HUMAN_VOCAL not in labels or (guard_only and len(labels) < 2):
        raise ValueError("The Human vocal recording guard and at least one visible class are required")
    nonbirds = frozenset((WORLDWIDE / "nonbird-labels.txt").read_text().splitlines()) | SOUNDS
    return f"""// Generated from the ordered model labels and recording policy.
#pragma once

namespace birdnet_species {{
inline constexpr int kCount = {len(labels)};
inline constexpr int kHumanVocal = {labels.index(HUMAN_VOCAL)};
inline constexpr char kDeployment[] = {json.dumps(deployment)};
inline constexpr bool kVocalGuardOnly = {str(guard_only).lower()};
inline constexpr unsigned char kSounds[] = {{{bitset(labels, SOUNDS)}}};
inline constexpr unsigned char kNonBirds[] = {{{bitset(labels, nonbirds)}}};

inline constexpr bool Visible(int index) {{
    return index >= 0 && index < kCount && (!kVocalGuardOnly || index != kHumanVocal);
}}
inline constexpr bool IsBird(int index) {{
    return index >= 0 && index < kCount && !(kNonBirds[index / 8] & (1u << (index % 8)));
}}
inline constexpr bool CanTriggerAudio(int index) {{
    return index >= 0 && index < kCount && !(kSounds[index / 8] & (1u << (index % 8)));
}}
}} // namespace birdnet_species
"""


def read_esp_profile(models):
    if models.resolve() == DEFAULT_MODELS.resolve():
        read_pinned(DEFAULT_MODELS / "esp32-packed.tflite")
        labels = load_labels(read_pinned(DEFAULT_MODELS / "labels.txt"))
        return render_profile(labels, False, "XIAO-Western-US-1018")
    label_data = (models / "labels.txt").read_bytes()
    packed = (models / "esp32-packed.tflite").read_bytes()
    meta = json.loads((models / "esp32-contract.json").read_text())
    if sha256(packed) != meta["model_sha256"] or sha256(label_data) != meta["labels_sha256"]:
        raise ValueError("XIAO model or ordered labels differ from their custom contract")
    labels = load_labels(label_data)
    guard_only = meta["vocal_guard_only"]
    if len(labels) != meta["classes"] or not isinstance(guard_only, bool):
        raise ValueError("Invalid XIAO class count or recording policy")
    policy = sha256(label_data + bytes([guard_only]))[:12]
    return render_profile(labels, guard_only, "XIAO-custom-" + policy)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", type=Path)
    parser.add_argument("header", type=Path)
    args = parser.parse_args()
    try:
        args.header.write_text(read_esp_profile(args.models))
    except (OSError, KeyError, ValueError) as error:
        parser.exit(1, f"{error}\n")
