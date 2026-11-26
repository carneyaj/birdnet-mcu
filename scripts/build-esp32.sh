#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SDK="${IDF_PATH:-$ROOT/.deps/esp-idf}"
export IDF_TOOLS_PATH="${IDF_TOOLS_PATH:-$ROOT/.deps/esp-tools}"
export IDF_COMPONENT_MANAGER=0
diagnostics=OFF
name=esp32
if [[ "${1:-}" == "--diagnostics" ]]; then
    diagnostics=ON
    name=esp32-diagnostics
    shift
fi
if [[ $# != 0 ]]; then
    echo "Usage: $0 [--diagnostics]" >&2
    exit 2
fi
if [[ ! -f "$SDK/export.sh" ]]; then
    echo "Run scripts/install-build-deps.sh esp32 or set IDF_PATH to ESP-IDF v5.4.3." >&2
    exit 1
fi
source "$SDK/export.sh" >/dev/null
if [[ "$(idf.py --version)" != "ESP-IDF v5.4.3" ]]; then
    echo "This firmware uses ESP-IDF v5.4.3." >&2
    exit 1
fi

project="$ROOT/firmware/esp32"
build="$ROOT/build/$name/obj"
out="$ROOT/build/$name"
mkdir -p "$build"
idf.py -C "$project" -B "$build" -G "Unix Makefiles" \
    -D "SDKCONFIG=$build/sdkconfig" -D "BIRDNET_DIAGNOSTICS=$diagnostics" build
image="$out/xiao-esp32s3.bin"
(cd "$build" && python -m esptool --chip esp32s3 merge_bin -o "$image" @flash_args)
python "$ROOT/tools/write_esp_manifest.py" "$image"
echo "Flash with python flash_esp32.py --bundle $out --port PORT"
