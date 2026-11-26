set(shared "${CMAKE_CURRENT_LIST_DIR}/../../common")
set(models "${CMAKE_CURRENT_LIST_DIR}/../../../models/western-us-v2")
set(MODEL_FILE "${models}/esp32-packed.tflite")
set(LABELS_FILE "${models}/labels.txt")
if(NOT CMAKE_SCRIPT_MODE_FILE)
    configure_file(model.S.in model.S @ONLY)
    set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/model.S" PROPERTIES
        OBJECT_DEPENDS "${MODEL_FILE};${LABELS_FILE}")
endif()

idf_component_register(
    SRCS "${CMAKE_CURRENT_BINARY_DIR}/model.S" app.cpp frontend.cpp packed_model.cpp swish.cpp
        stem_pool.cpp oled.cpp recorder.cpp
        "${shared}/birdnet_opus_codec.cpp" "${shared}/birdnet_frontend_assets.cpp"
    INCLUDE_DIRS "." "${shared}"
    REQUIRES espressif__esp-nn esp-tflite-micro esp-dsp esp_timer esp_psram esp_pm
        esp_driver_i2s esp_driver_i2c fatfs esp_driver_sdspi esp_driver_sdmmc
        esp_driver_spi esp_driver_gpio esp_hw_support
)
set_source_files_properties(frontend.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
include(opus.cmake)
if(BIRDNET_DIAGNOSTICS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE BIRDNET_DIAGNOSTICS=1)
endif()
