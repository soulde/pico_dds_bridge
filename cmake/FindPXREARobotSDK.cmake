# FindPXREARobotSDK.cmake
#
# Input hints:
#   -DPXREA_ROOT=/path/to/prefix
#   environment variable PXREA_ROOT
#
# Exports:
#   PXREARobotSDK_FOUND
#   PXREARobotSDK_INCLUDE_DIR
#   PXREARobotSDK_LIBRARY
#   PXREA::RobotSDK

set(_PXREA_HINTS
    "${PXREA_ROOT}"
    "$ENV{PXREA_ROOT}"
    "${CMAKE_CURRENT_LIST_DIR}/../.deps/xrobotoolkit"
)

find_path(PXREARobotSDK_INCLUDE_DIR
    NAMES PXREARobotSDK.h
    HINTS ${_PXREA_HINTS}
    PATH_SUFFIXES include
)

find_library(PXREARobotSDK_LIBRARY
    NAMES PXREARobotSDK libPXREARobotSDK
    HINTS ${_PXREA_HINTS}
    PATH_SUFFIXES lib lib64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PXREARobotSDK
    REQUIRED_VARS
        PXREARobotSDK_INCLUDE_DIR
        PXREARobotSDK_LIBRARY
)

if(PXREARobotSDK_FOUND AND NOT TARGET PXREA::RobotSDK)
    add_library(PXREA::RobotSDK SHARED IMPORTED)
    set_target_properties(PXREA::RobotSDK PROPERTIES
        IMPORTED_LOCATION "${PXREARobotSDK_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PXREARobotSDK_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(
    PXREARobotSDK_INCLUDE_DIR
    PXREARobotSDK_LIBRARY
)
