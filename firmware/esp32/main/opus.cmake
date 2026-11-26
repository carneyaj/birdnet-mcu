# Cross-compile the same pinned floating-point encoder used by the Grove app.
include(ExternalProject)
set(archive "${CMAKE_CURRENT_LIST_DIR}/../../../licenses/opus-1.6.1.tar.gz")
set(opus_build "${CMAKE_CURRENT_BINARY_DIR}/opus")
ExternalProject_Add(xiao_opus
    URL "${archive}"
    URL_HASH SHA256=6ffcb593207be92584df15b32466ed64bbec99109f007c82205f0194572411a1
    SOURCE_DIR "${opus_build}/source" BINARY_DIR "${opus_build}/build"
    CMAKE_ARGS -DCMAKE_SYSTEM_NAME=Generic -DCMAKE_SYSTEM_PROCESSOR=xtensa
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY
        "-DCMAKE_C_FLAGS=-mlongcalls -ffunction-sections -fdata-sections"
        -DCMAKE_BUILD_TYPE=MinSizeRel -DBUILD_SHARED_LIBS=OFF
        -DOPUS_DISABLE_INTRINSICS=ON -DOPUS_FIXED_POINT=OFF -DOPUS_STACK_PROTECTOR=OFF
        -DOPUS_FORTIFY_SOURCE=OFF -DOPUS_DRED=OFF -DOPUS_OSCE=OFF
        -DOPUS_BUILD_TESTING=OFF -DOPUS_BUILD_PROGRAMS=OFF
    BUILD_BYPRODUCTS "${opus_build}/build/libopus.a" INSTALL_COMMAND "")
add_dependencies(${COMPONENT_LIB} xiao_opus)
target_compile_definitions(${COMPONENT_LIB} PRIVATE BIRDNET_DETECTION_AUDIO_CODEC=2)
target_include_directories(${COMPONENT_LIB} PRIVATE "${opus_build}/source/include")
target_link_libraries(${COMPONENT_LIB} PRIVATE "${opus_build}/build/libopus.a")
