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
set(CHRONO_PACKAGE_DIR
    "/home/chrono-user/mountdir/packages"
    CACHE PATH "Chrono third-party package directory in the container")
set(OPTIX_INSTALL_DIR
    "${CHRONO_PACKAGE_DIR}/optix"
    CACHE PATH "OptiX SDK install directory")
set(OPTIX_INCLUDE_DIR
    "${OPTIX_INSTALL_DIR}/include"
    CACHE PATH "OptiX SDK include directory")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(AMD_UW_ENABLE_ROS2 "Enable ROS2 command/state bridge" ON)
option(AMD_UW_USE_SOLIDWORKS_IMPORTER "Build the LRV arm from the Chrono SolidWorks Python importer" OFF)

add_executable(demo_SYN_polaris_flat
    main.cpp
    src/LrvArm.cpp
    src/MaterialUtils.cpp
    src/RobotRig.cpp
    src/RockField.cpp
    src/SynAgents.cpp)

if(AMD_UW_ENABLE_ROS2)
    find_package(ament_cmake QUIET)
    find_package(rclcpp QUIET)
    find_package(std_msgs QUIET)

    if(NOT rclcpp_FOUND OR NOT std_msgs_FOUND)
        message(FATAL_ERROR "AMD_UW_ENABLE_ROS2=ON requires ROS2 rclcpp and std_msgs. Source ROS2 or configure with -DAMD_UW_ENABLE_ROS2=OFF.")
    endif()

    target_sources(demo_SYN_polaris_flat PRIVATE src/RosArmBridge.cpp src/RosControllerDriver.cpp)
    target_compile_definitions(demo_SYN_polaris_flat PRIVATE AMD_UW_ENABLE_ROS2)
    message(STATUS "ROS2 bridge enabled")
endif()

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
        "CHRONO_DATA_DIR=\"${CHRONO_BUILD_DIR}/data/\""
        "CHRONO_VEHICLE_DATA_DIR=\"${CHRONO_BUILD_DIR}/bin/data/vehicle/\""
        "UW_AMD_DATA_DIR=\"${UW_AMD_DATA_DIR}\""
        "UW_AMD_VEHICLE_DATA_DIR=\"${UW_AMD_VEHICLE_DATA_DIR}\"")

if(AMD_UW_USE_SOLIDWORKS_IMPORTER)
    target_compile_definitions(demo_SYN_polaris_flat
        PRIVATE
            AMD_UW_USE_SOLIDWORKS_IMPORTER
            # So the embedded interpreter can `import pychrono` without the caller
            # having to export PYTHONPATH; the bindings live in the Chrono build's bin/.
            "PYCHRONO_MODULE_DIR=\"${CHRONO_BUILD_DIR}/bin\"")
    message(STATUS "LRV arm SolidWorks importer enabled")
endif()

target_include_directories(demo_SYN_polaris_flat
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CHRONO_ROOT}/src"
        "${CHRONO_BUILD_DIR}"
        "${CHRONO_ROOT}/src/chrono/collision/bullet"
        "${CHRONO_ROOT}/src/chrono_thirdparty/HACDv2"
        "${CHRONO_ROOT}/src/chrono_thirdparty/yaml-cpp/include"
        "${CHRONO_ROOT}/src/chrono_thirdparty/flatbuffers/include"
        "${OPTIX_INCLUDE_DIR}")

target_include_directories(demo_SYN_polaris_flat
    SYSTEM PRIVATE
        /usr/include/eigen3
        /usr/local/cuda/include
        /usr/local/cuda/include/cccl
        "${CHRONO_PACKAGE_DIR}/vsg/include"
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
        "-Wl,-rpath,${CHRONO_BUILD_DIR}/lib:${CHRONO_PACKAGE_DIR}/vsg/lib:/usr/lib/x86_64-linux-gnu/openmpi/lib::::"
        "-Wl,-rpath-link,${CHRONO_PACKAGE_DIR}/vsg/lib")

target_link_libraries(demo_SYN_polaris_flat
        # SolidWorks importer only: libChrono_parsers.so statically embeds and
        # globally re-exports its own tinyxml2. FastDDS (the ROS2 rmw) links the
        # system libtinyxml2.so.9 and parses XML profiles at DDS-participant/node
        # creation. With parsers loaded, ELF global-symbol interposition binds
        # FastDDS's tinyxml2 calls to Chrono's ABI-incompatible copy, so a virtual
        # call lands on a bad vtable slot and jumps to 0x0 (SIGSEGV during
        # rclcpp::Node creation). Force the system tinyxml2 to load first so it
        # wins global resolution; the SolidWorks import path never calls tinyxml2
        # (that is only Chrono's URDF parser), so Chrono is unaffected.
        # --no-as-needed keeps the (directly unreferenced) lib in DT_NEEDED.
        "$<$<BOOL:${AMD_UW_USE_SOLIDWORKS_IMPORTER}>:-Wl,--no-as-needed>"
        "$<$<BOOL:${AMD_UW_USE_SOLIDWORKS_IMPORTER}>:/usr/lib/x86_64-linux-gnu/libtinyxml2.so.9>"
        "$<$<BOOL:${AMD_UW_USE_SOLIDWORKS_IMPORTER}>:-Wl,--as-needed>"
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
        "${CHRONO_PACKAGE_DIR}/vsg/lib/libvsgXchange.so.1.1.12"
        "${CHRONO_PACKAGE_DIR}/vsg/lib/libvsgImGui.so.0.7.0"
        "${CHRONO_PACKAGE_DIR}/vsg/lib/libvsg.so.1.1.15"
        /usr/lib/x86_64-linux-gnu/libvulkan.so
        /usr/lib/x86_64-linux-gnu/libIrrlicht.so
        /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi_cxx.so
        /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so)

if(AMD_UW_USE_SOLIDWORKS_IMPORTER)
    target_link_libraries(demo_SYN_polaris_flat "${CHRONO_BUILD_DIR}/lib/libChrono_parsers.so")
endif()

if(AMD_UW_ENABLE_ROS2)
    if(ament_cmake_FOUND)
        ament_target_dependencies(demo_SYN_polaris_flat rclcpp std_msgs)
    else()
        target_link_libraries(demo_SYN_polaris_flat rclcpp::rclcpp ${std_msgs_TARGETS})
    endif()
endif()
