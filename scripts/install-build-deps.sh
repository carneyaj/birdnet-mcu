#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
deps="$ROOT/.deps"
mkdir -p "$deps"

case "${1:-}" in
    grove)
        sdk="$deps/grove-sdk"
        revision=933810ccd5deb1adf2ae8005a8dfca78fc0cb63a
        if [[ ! -d "$sdk/.git" ]]; then
            git clone https://github.com/HimaxWiseEyePlus/Seeed_Grove_Vision_AI_Module_V2.git "$sdk"
            git -C "$sdk" checkout --detach "$revision"
        fi
        if [[ "$(git -C "$sdk" rev-parse HEAD)" != "$revision" ]]; then
            echo "Expected Himax SDK revision $revision in $sdk." >&2
            exit 1
        fi
        echo "Himax SDK ready. Install Arm GNU 15.2.Rel1 arm-none-eabi before building."
        ;;
    esp32)
        sdk="$deps/esp-idf"
        revision=ea1c174c1cbb7348bd8ba0ff1eb306246938dd80
        if [[ ! -d "$sdk/.git" ]]; then
            git clone --branch v5.4.3 --single-branch https://github.com/espressif/esp-idf.git "$sdk"
        fi
        if [[ "$(git -C "$sdk" rev-parse HEAD)" != "$revision" ]]; then
            echo "Expected ESP-IDF revision $revision in $sdk." >&2
            exit 1
        fi
        git -C "$sdk" submodule update --init --recursive
        export IDF_TOOLS_PATH="$deps/esp-tools"
        "$sdk/install.sh" esp32s3
        ;;
    *) echo "Usage: $0 grove|esp32" >&2; exit 2 ;;
esac
