cmake_minimum_required(VERSION 3.22)

project(pico_dds_bridge
    VERSION 0.1.0
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PICO_DDS_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Threads REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(PXREARobotSDK REQUIRED)

# Generate CycloneDDS C type-support from IDL.
idlc_generate(
    TARGET pico_tracking_idl
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/idl/pico_tracking.idl"
    WARNINGS no-implicit-extensibility
)

add_library(pico_bridge_core
    src/coordinate_transform.cpp
    src/parser.cpp
    src/xrobotoolkit_client.cpp
    src/dds_publisher.cpp
)

target_include_directories(pico_bridge_core
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(pico_bridge_core
    PUBLIC
        CycloneDDS::ddsc
        pico_tracking_idl
        nlohmann_json::nlohmann_json
        PXREA::RobotSDK
        Threads::Threads
)

target_compile_features(pico_bridge_core PUBLIC cxx_std_20)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(pico_bridge_core PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow
    )
    if(PICO_DDS_WARNINGS_AS_ERRORS)
        target_compile_options(pico_bridge_core PRIVATE -Werror)
    endif()
endif()

add_executable(pico_dds_bridge src/main.cpp)
target_link_libraries(pico_dds_bridge PRIVATE pico_bridge_core)

# Make the locally-installed vendor SDK discoverable at runtime without
# forcing users to modify /etc/ld.so.conf.
if(PXREARobotSDK_LIBRARY)
    get_filename_component(_pxrea_lib_dir "${PXREARobotSDK_LIBRARY}" DIRECTORY)
    set_target_properties(pico_dds_bridge PROPERTIES
        BUILD_RPATH "${_pxrea_lib_dir}"
        INSTALL_RPATH "${_pxrea_lib_dir}"
    )
endif()

include(GNUInstallDirs)
install(TARGETS pico_dds_bridge
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(FILES idl/pico_tracking.idl DESTINATION "${CMAKE_INSTALL_DATADIR}/pico_dds_bridge/idl")
