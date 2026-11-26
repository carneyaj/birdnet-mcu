set(shared "${CMAKE_CURRENT_LIST_DIR}/../../common")
set(repo "${CMAKE_CURRENT_LIST_DIR}/../../..")
set(models "${repo}/models/western-us-v2")
if(BIRDNET_MODELS_DIR)
    set(models "${BIRDNET_MODELS_DIR}")
endif()
set(profile_header "${CMAKE_CURRENT_BINARY_DIR}/birdnet_species_profile.h")
set(MODEL_FILE "${models}/esp32-packed.tflite")
set(LABELS_FILE "${models}/labels.txt")
if(NOT CMAKE_SCRIPT_MODE_FILE)
    idf_build_get_property(python PYTHON)
    execute_process(
        COMMAND "${python}" "${repo}/tools/species_profile.py" "${models}" "${profile_header}"
        RESULT_VARIABLE profile_result)
    if(NOT profile_result EQUAL 0)
        message(FATAL_ERROR "Could not generate the XIAO species policy")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${MODEL_FILE}" "${LABELS_FILE}" "${repo}/models/worldwide-v2/nonbird-labels.txt"
        "${repo}/tools/species_profile.py" "${repo}/models/SHA256SUMS")
    if(BIRDNET_MODELS_DIR)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${models}/esp32-contract.json")
    endif()
    configure_file(model.S.in model.S @ONLY)
    set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/model.S" PROPERTIES
        OBJECT_DEPENDS "${MODEL_FILE};${LABELS_FILE}")
endif()

idf_component_register(
    SRCS "${CMAKE_CURRENT_BINARY_DIR}/model.S" app.cpp frontend.cpp packed_model.cpp swish.cpp
        stem_pool.cpp oled.cpp recorder.cpp
        "${shared}/birdnet_opus_codec.cpp" "${shared}/birdnet_frontend_assets.cpp"
    INCLUDE_DIRS "." "${shared}" PRIV_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}"
    REQUIRES espressif__esp-nn esp-tflite-micro esp-dsp esp_timer esp_psram esp_pm
        esp_driver_i2s esp_driver_i2c fatfs esp_driver_sdspi esp_driver_sdmmc
        esp_driver_spi esp_driver_gpio esp_hw_support
)
set_source_files_properties(frontend.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
include(opus.cmake)
if(BIRDNET_DIAGNOSTICS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE BIRDNET_DIAGNOSTICS=1)
endif()
