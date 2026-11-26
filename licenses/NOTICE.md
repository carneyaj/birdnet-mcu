# Attribution and licenses

## BirdNET models

BirdNET v2.4 is by the BirdNET team at the Cornell Lab of Ornithology and Chemnitz University of Technology. This project adapts the BirdNET acoustic model by separating its frontend, applying quantization-aware training, slicing the classifier for a regional profile, and compiling or packing the result for two microcontrollers. These adapted model artifacts are licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International. This includes over-the-air or embedded copies of the model distributed with firmware. [BirdNET Analyzer attribution and model license](https://github.com/birdnet-team/BirdNET-Analyzer#license) · [CC BY-NC-SA 4.0 legal code](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)

## Applied libraries and SDKs

- Grove uses the [Himax Seeed Grove Vision AI V2 SDK](https://github.com/HimaxWiseEyePlus/Seeed_Grove_Vision_AI_Module_V2/tree/933810ccd5deb1adf2ae8005a8dfca78fc0cb63a) with the included [SDK patch](../firmware/grove/sdk.patch). See [Himax’s repository license](Himax.txt) and the notices for its underlying components in the SDK.
- XIAO uses [ESP-IDF 5.4.3](https://github.com/espressif/esp-idf/tree/v5.4.3), tuned copies of [ESP-NN 1.5.0](../firmware/esp32/components/espressif__esp-nn/LICENSE) and [esp-tflite-micro 1.4.3](../firmware/esp32/components/esp-tflite-micro/LICENSE), and [ESP-DSP](../firmware/esp32/components/esp-dsp/LICENSE) at revision `921baa14ce8120831cdc1ddafb2d1f40ca1ea43f`. The platforms retain their upstream notices in the dependency trees.
- Both firmware images link Opus 1.6.1 from Xiph.Org. [Opus license](Opus.txt) · [Source release](https://downloads.xiph.org/releases/opus/opus-1.6.1.tar.gz)
- The XIAO display font carries the Adafruit license in [oled_font.h](../firmware/esp32/main/oled_font.h).
