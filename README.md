# BirdNET V2 int8 field firmware

## Flash and run

Use a **Seeed Grove Vision AI Module V2** or a **Seeed XIAO ESP32S3 Sense** with its microphone daughterboard attached. Each runs the Western US model with 1,018 classes. Connect one board over USB with a data cable and close any serial monitor before flashing.

### 1. Set up the flasher

From this folder, with Python 3.10 or newer:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-flash.txt
python -m serial.tools.list_ports -v
```

On Windows PowerShell, use `py -m venv .venv` and `.venv\Scripts\Activate.ps1`. Connect and disconnect the board to identify its port. Substitute its actual port for `PORT` below, such as `/dev/cu.usbmodem...`, `/dev/ttyACM0`, or `COM5`.

### 2. Flash your board

**Grove Vision AI V2**

```sh
python flash_grove.py --port PORT
```

Press **RESET** when prompted. Keep USB connected until the flasher reports that the board is rebooting. The serial transfer sends the paired CM55M/CM55S firmware first and then the 7.3 MB model and label payload. With the default paced XMODEM transfer, allow about 20 minutes. Retry the same command and press RESET again if a transfer is interrupted. [Himax bootloader guide](https://github.com/HimaxWiseEyePlus/Seeed_Grove_Vision_AI_Module_V2#flash-image-update-at-linux-environment-by-python-code)

**XIAO ESP32S3 Sense**

```sh
python flash_esp32.py --port PORT
```

If the ESP32 does not enter download mode, hold **BOOT**, tap **RESET**, release **BOOT**, and check the port again. Run the command again, optionally with `--baud 115200` for an unreliable USB connection. Tap RESET after flashing if the board stays in download mode. The single image includes the bootloader, partition table, app, model and labels. [Seeed XIAO guide](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)

Both flashers accept `--verify` to check a bundle offline and `--list` to find ports. Flashing replaces the board’s current firmware. Card contents are separate.

### 3. Check capture and collect results

| Board | First check | Card output |
| --- | --- | --- |
| Grove | Open `python -m serial.tools.miniterm PORT 921600` and tap RESET. Look for the `[bnrelease]` deployment line. The detection LED reports accepted hits. Per-window logging is available in the diagnostic build. | `BIRDLOG.CSV`, `AUDIOMAP.CSV`, periodic `HEALTH.CSV`, per-session manifests, and Ogg Opus clips under `AUDIO/`. |
| XIAO | Open `python -m serial.tools.miniterm PORT 115200`. Look for `ready, 1018 classes` followed by one `window=` line per processed window. An optional expansion OLED displays the current hit. | `HITS.CSV` and detection-triggered `.OGG` clips in the card root. |

Use a FAT32 microSD card for persistent results. Both boards classify with or without a card. Play a known local call to exercise the recording path and wait about a minute for the Grove’s CSV flush. Grove users can download files through USB with `python tools/grove_sd_download.py --port PORT list`, or recover files by powering down and removing the card. The XIAO tries the expansion base slot first and then the Sense slot. On an expansion base the OLED uses I²C address `0x3C`, D4/SDA and D5/SCL. [Grove board](https://wiki.seeedstudio.com/grove_vision_ai_v2/) · [XIAO expansion base](https://wiki.seeedstudio.com/Seeeduino-XIAO-Expansion-Board/)

For the optional Grove peripherals, use the 3.3 V header with GND, 3V3, D4/PA3 (SDA), and D5/PA2 (SCL). The field firmware uses the 128×64 SSD1306 at `0x3C`, SHT4x at `0x44`, and MAX17048 at `0x36`. A brief BOOT-button press during normal operation wakes the status display. [Grove schematic](https://files.seeedstudio.com/wiki/grove-vision-ai-v2/Grove_Vision_AI_Module_V2_Circuit_Diagram.pdf)

## Build the firmware

The build scripts produce flash bundles under `build/`. The shipping images live under `prebuilt/`.

| Target | Build dependencies |
| --- | --- |
| Grove | Bash, Git, Python 3, GNU Make, rsync, Arm GNU Toolchain **15.2.Rel1** for `arm-none-eabi`, and the pinned Himax SDK/image generator. The SDK supplies CMSIS-DSP, CMSIS-NN, TFLM, the Ethos-U driver, and FatFs. The repo includes the matching Cortex-M55 Opus 1.6.1 library. The image generator supports Apple silicon macOS and x86-64 Linux. |
| XIAO | Bash, Git, Python 3, make, CMake, and **ESP-IDF 5.4.3** with the ESP32-S3 toolchain. The repo carries the tuned ESP-NN 1.5.0 and esp-tflite-micro 1.4.3 sources, a pinned ESP-DSP snapshot, and the Opus 1.6.1 source archive. |

On macOS install the Xcode command-line tools and CMake. On Debian/Ubuntu install `build-essential git cmake rsync python3 python3-venv flex bison gperf libffi-dev libssl-dev libusb-1.0-0`. Install Arm GNU separately for the Grove and put its compiler on `PATH` or set `ARM_GNU_BIN` to its `bin` directory. Keep the Grove checkout path free of spaces. [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) · [ESP-IDF prerequisites](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/get-started/)

```sh
./scripts/install-build-deps.sh grove
./scripts/build-grove.sh

./scripts/install-build-deps.sh esp32
./scripts/build-esp32.sh
```

To flash a local build, run `python flash_grove.py --bundle build/grove --port PORT` or `python flash_esp32.py --bundle build/esp32 --port PORT`. Each build script also accepts `--diagnostics`. Pass `build/grove-diagnostics` or `build/esp32-diagnostics` to the flasher for those builds. Grove diagnostics enable per-window prediction and status messages. XIAO diagnostics add frontend, inference, end-to-end timing and DMA-overrun counters. Normal builds still report startup and field errors.

The SDK installer fetches the pinned vendor sources under `.deps/`. Grove applies the included SDK patch in that checkout for the app entry, SD recovery, Cortex-M55 timer ownership, inference support, and host make compatibility. XIAO builders can point `IDF_PATH` and `IDF_TOOLS_PATH` at an existing 5.4.3 install. Use `IDF_PYTHON_ENV_PATH` when that install uses a specific Python environment. The checked-in model assets and frontend tables are sufficient for both firmware builds.

## Build a custom species list

Start with names from the [worldwide V2 catalog](models/worldwide-v2/labels.txt), including species outside the Western US field model. Put one common name, scientific name, or full `Scientific name_Common name` label on each line. When a name matches multiple classes, use the full label. Lines starting with `#` and blank lines are ignored.

For example, save this as `species.txt`:

```text
Great Horned Owl
Apteryx mantelli_North Island Brown Kiwi
Cardinalis cardinalis
```

With Python 3.12 and the firmware build tools above, generate the model and build the images:

```sh
python3.12 -m venv .deps/model-tools
.deps/model-tools/bin/python -m pip install -r requirements-models.txt
.deps/model-tools/bin/python tools/custom_species.py species.txt --output build/my-species
./scripts/build-grove.sh --models build/my-species
./scripts/build-esp32.sh --models build/my-species
```

Flash the matching custom bundle:

```sh
python flash_grove.py --bundle build/my-species/grove --port PORT
python flash_esp32.py --bundle build/my-species/esp32 --port PORT
```

Generate a fresh output directory for each list. The generator writes the reduced portable model, the ordered `labels.txt`, and `species-selection.json` with the selected labels and source indices. Both compilers use this same output order. To prepare model files for only one device, add `--board grove` or `--board esp32` to the generator command. Firmware build checks enforce each device’s memory and flash limits. Custom Grove models use stock Vela 5.2.0, while the prebuilt Western US model uses its tuned Vela export. [Arm Vela documentation](https://arm-software.github.io/CMSIS-Ethos-U/main/vela/index.html)

The generator copies the selected int8 classifier rows and verifies their scores against the worldwide model on three host inputs. It always includes the Human vocal class for the 0.30 recording veto. If the list omits that class, the firmware keeps it out of normal detections. The bundled prebuilt images continue to provide the Western US selection.

## Model and relationship to BirdNET

| File | Purpose |
| --- | --- |
| [Worldwide V2 int8 model](models/worldwide-v2/model_int8.tflite) and [labels](models/worldwide-v2/labels.txt) | The complete 6,522-output V2 parent before any board-specific conversion. |
| [Western US int8 model](models/western-us-v2/model_int8.tflite) and [labels](models/western-us-v2/labels.txt) | A standard TFLite slice with 1,000 bird classes, eight mammals, and ten other sounds. Coverage includes the western states, Alaska and Hawaii. |
| [Grove Ethos-U model](models/western-us-v2/grove-ethosu.tflite) | Vela-compiled version for the WE2 NPU. The Grove bundle appends the ordered label table and flashes that payload at `0x200000`. |
| [XIAO packed model](models/western-us-v2/esp32-packed.tflite) | The regional graph with custom ESP32 operator fusions and flash-packed weights. The firmware links it and the ordered labels into the app image. |

Here **V2** is the project’s quantization-aware BirdNET v2.4 derivative. It retains the BirdNET backbone architecture, two-resolution spectrogram design, nominal 48 kHz mono input, 144,000-sample windows and independent sigmoid-transformed class logits. The desktop v2.4 acoustic model accepts waveform samples and contains the frontend inside its graph. The microcontroller runtime runs that frontend in firmware and passes a signed int8 `[1, 96, 511, 2]` feature tensor to the model. The weights were tuned for int8 with teacher supervision. Both prebuilt board targets use the same Western US output order and preserve the selected classifier rows and output quantization from the worldwide int8 parent. [Upstream BirdNET model documentation](https://github.com/birdnet-team/BirdNET-Analyzer/blob/main/docs/models.rst) · [Quantization audit](models/western-us-v2/quantization-audit.json)

The desktop Analyzer also offers file input, overlap, batch processing, and geographic or seasonal filtering. This firmware listens to the local microphone in consecutive three-second nominal windows and uses the label set built into its installed model. Training, microphone response, and arithmetic can all change scores relative to upstream BirdNET. A score of 0.25 is the field trigger, and a Human vocal score of at least 0.30 suppresses audio saving. Grove can log multiple above-threshold classes and collects hourly ambient clips. XIAO reports the top live class and records at most the highest-scoring eligible animal per window. Other sound classes can be reported without triggering audio.

The V2 adaptation follows BirdNET’s **CC BY-NC-SA 4.0** model license, including the model bytes embedded in the ESP32 image and bundled with the Grove. See [Attribution and licenses](licenses/NOTICE.md) and [model checksums](models/SHA256SUMS).

## Release verification

The bundled images were rebuilt from these sources and verified on the build host. Grove checks cover linker placement, included Opus code, the embedded firmware identity, the paired model and label payload, and six host tests for capture, field storage and the human-vocal veto. XIAO checks cover the merged flash image, partition fit, embedded model and label bytes, and the application image checksum. The custom build path was also checked on both targets with a list containing North Island Brown Kiwi. The earlier field baselines ran on hardware. These rebuilt images still need a board smoke test before a field rollout.

## Implementation and streamlining

In prior hardware runs of the same regional profile, Grove took medians of 82 ms for the frontend and 178 ms for inference during acoustic replay. XIAO’s median capture-to-result latency was 1.516 s over 36 live windows. These measurements describe the previous field builds.

### Grove Vision AI V2

The CM55S capture helper keeps PDM DMA running and hands contiguous windows to the CM55M through a shared mailbox. The configured clock yields about 46,875 samples per wall-clock second, so a nominal 144,000-sample window spans about 3.072 seconds. CM55M builds the feature tensor and invokes the Ethos-U55, then returns to audio-only sleep. The production path uses Cortex-M55 Helium FFT and mel kernels, a half-rate wide-window branch, precomputed reorder and compression tables, and a 1,088 KiB tensor arena. It keeps the main sleep retention to one HSC SRAM bank and retains compact counters while reading label strings from flash. The field profile saves Ogg Opus clips and stages flash-resident labels in RAM before FatFs writes. Browse the [capture helper](firmware/grove/capture), [application](firmware/grove/app/birdnet_we2.cpp), [field profile](firmware/grove/app/birdnet_we2.mk), and [inference pipeline](firmware/grove/app/inference_pipeline.cpp).

For the release, the two cores build together under one fixed field profile. Tensor dumps, PCM probes, memory tests, and hardware comparison branches were removed. Status and prediction verbosity are opt-in through the diagnostic build. SD export, LED/OLED status, watchdog recovery, field logging, and the optional SHT41 and MAX17048 integrations remain part of the field profile.

### XIAO ESP32S3 Sense

I²S PDM captures on GPIO42 clock and GPIO41 data. A high-priority task fills window buffers while the classifier processes the newest complete window. A separate task mounts the card, encodes Opus, and returns the audio buffer. If it still owns the spare when the next hit arrives, classification continues and the serial result reports `record=busy`. ESP-DSP packs both real FFT branches into one radix-4 transform and splits frame work across the cores. Quantization-boundary cases use higher-precision evaluation. ESP-NN uses S3 SIMD, dual-core convolution, aligned and padded weight layouts, and fused activation lookup tables. The firmware expands the large packed constants into octal PSRAM and uses a 2 MiB TFLM arena. Dynamic frequency scaling reduces CPU frequency when both cores are idle and keeps the microphone clock running. Browse the [capture and classification loop](firmware/esp32/main/app.cpp), [frontend](firmware/esp32/main/frontend.cpp), and [recorder](firmware/esp32/main/recorder.cpp).

For the release, the boot fixture, operator profiler, PCM probe and alternate board paths were removed. The firmware starts live capture after validating the model contract. Additional timing is isolated in [the diagnostics module](firmware/esp32/main/diagnostics/timing.h), while the normal serial output shows the class, score and recording outcome.
