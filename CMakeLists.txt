cmake_minimum_required(VERSION 3.18)

project(amd_uw_synchrono_demos LANGUAGES CXX)

set(CHRONO_ROOT
    "/home/chrono-user/mountdir/chrono"
    CACHE PATH "Chrono source tree in the container")
set(CHRONO_BUILD_DIR
    "${CHRONO_ROOT}/build"
    CACHE PATH "Chrono build tree in the container")
set(UW_AMD_DATA_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/data"
    CACHE PATH "AMD-UW data directory")
set(UW_AMD_VEHICLE_DATA_DIR
    "${UW_AMD_DATA_DIR}/vehicle"
    CACHE PATH "AMD-UW Chrono::Vehicle data directory")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(demo_SYN_polaris_flat syn_polaris_flat.cpp)

target_compile_definitions(demo_SYN_polaris_flat
    PRIVATE
        CHRONO_USE_CUDA
        CH_API_COMPILE_SENSOR
        CH_IGNORE_DEPRECATED
        THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA
        THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP
        VSG_SHARED_LIBRARY
        VSGXCHANGE_SHARED_LIBRARY
        VSGIMGUI_SHARED_LIBRARY
        USE_SENSOR_GLFW
        _ENABLE_EXTENDED_ALIGNED_STORAGE
        "CHRONO_VEHICLE_DATA_DIR=\"${CHRONO_BUILD_DIR}/bin/data/vehicle/\""
        "UW_AMD_DATA_DIR=\"${UW_AMD_DATA_DIR}\""
        "UW_AMD_VEHICLE_DATA_DIR=\"${UW_AMD_VEHICLE_DATA_DIR}\"")

target_include_directories(demo_SYN_polaris_flat
    PRIVATE
        "${CHRONO_ROOT}/src"
        "${CHRONO_BUILD_DIR}"
        "${CHRONO_ROOT}/src/chrono/collision/bullet"
        "${CHRONO_ROOT}/src/chrono_thirdparty/HACDv2"
        "${CHRONO_ROOT}/src/chrono_thirdparty/yaml-cpp/include"
        "${CHRONO_ROOT}/src/chrono_thirdparty/flatbuffers/include"
        "/opt/optix/include")

target_include_directories(demo_SYN_polaris_flat
    SYSTEM PRIVATE
        /usr/include/eigen3
        /usr/local/cuda/include
        /usr/local/cuda/include/cccl
        /home/chrono-user/mountdir/packages/vsg/include
        /usr/include/irrlicht
        /usr/lib/x86_64-linux-gnu/openmpi/include
        /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi)

target_compile_options(demo_SYN_polaris_flat
    PRIVATE
        -march=native
        -Wint-in-bool-context
        -Wno-sign-compare
        -Wno-reorder
        -Wno-unused-function
        -Wno-unused-parameter
        -Wno-unused-result
        -Wno-deprecated
        -Wno-stringop-overflow
        -fopenmp)

target_link_options(demo_SYN_polaris_flat
    PRIVATE
        "-Wl,--disable-new-dtags"
        "-Wl,-rpath,${CHRONO_BUILD_DIR}/lib:/home/chrono-user/mountdir/packages/vsg/lib:/usr/lib/x86_64-linux-gnu/openmpi/lib::::"
        "-Wl,-rpath-link,/home/chrono-user/mountdir/packages/vsg/lib")

target_link_libraries(demo_SYN_polaris_flat
    PRIVATE
        "${CHRONO_BUILD_DIR}/lib/libChrono_synchrono.so"
        "${CHRONO_BUILD_DIR}/lib/libChronoModels_vehicle.so"
        "${CHRONO_BUILD_DIR}/lib/libChronoModels_robot.so"
        "${CHRONO_BUILD_DIR}/lib/libChrono_vehicle_vsg.so"
        "${CHRONO_BUILD_DIR}/lib/libChrono_fsisph_vsg.so"
        "${CHRONO_BUILD_DIR}/lib/libChrono_sensor.so"
        /usr/lib/x86_64-linux-gnu/libGLEW.so
        /usr/lib/x86_64-linux-gnu/libglfw.so.3.3
        /usr/lib/x86_64-linux-gnu/libGLX.so
        /usr/lib/x86_64-linux-gnu/libOpenGL.so
        "${CHRONO_BUILD_DIR}/lib/libChrono_vehicle_irrlicht.so"
        "${CHRONO_BUILD_DIR}/lib/libChrono_vehicle.so"
        "${CHRONO_BUILD_DIR}/lib/libChrono_fsisph.so"
        /usr/local/cuda/lib64/libcudart_static.a
        dl
        /usr/lib/x86_64-linux-gnu/librt.a
        /usr/local/cuda/targets/x86_64-linux/lib/stubs/libnvrtc.so
        /usr/local/cuda/targets/x86_64-linux/lib/stubs/libcuda.so
        /usr/local/cuda/targets/x86_64-linux/lib/stubs/libcublas.so
        /usr/local/cuda/targets/x86_64-linux/lib/stubs/libcusparse.so
        "${CHRONO_BUILD_DIR}/lib/libChrono_fsi.so"
        "${CHRONO_BUILD_DIR}/lib/libChrono_vsg.so"
        "${CHRONO_BUILD_DIR}/lib/libChrono_irrlicht.so"
        "${CHRONO_BUILD_DIR}/lib/libChrono_core.so"
        /usr/lib/gcc/x86_64-linux-gnu/11/libgomp.so
        /usr/lib/x86_64-linux-gnu/libpthread.a
        pthread
        "${CHRONO_BUILD_DIR}/lib/libyaml-cpp.so.0.8.0"
        /home/chrono-user/mountdir/packages/vsg/lib/libvsgXchange.so.1.1.12
        /home/chrono-user/mountdir/packages/vsg/lib/libvsgImGui.so.0.7.0
        /home/chrono-user/mountdir/packages/vsg/lib/libvsg.so.1.1.15
        /usr/lib/x86_64-linux-gnu/libvulkan.so
        /usr/lib/x86_64-linux-gnu/libIrrlicht.so
        /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi_cxx.so
        /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so)
