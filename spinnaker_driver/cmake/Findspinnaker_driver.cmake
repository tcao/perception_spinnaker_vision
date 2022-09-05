#
#  This utility is used to find custom spinnaker driver include/library paths
#  (Note: This is not for Spinnaker SDK)
#  By Ting Cao
#
#  SPINNAKER_DRIVER_FOUND       - spinnaker driver package found
#  SPINNAKER_DRIVER_INCLUDE_DIR - spinnaker drivernclude directories
#  SPINNAKER_DRIVER_LIBRARY_DIR - link these to use GStreamer
#
include(FindPackageHandleStandardArgs)

# When successful SPINNAKER_DRIVER_INCLUDE_DIR is set
find_path(SPINNAKER_DRIVER_INCLUDE_DIR
    NAMES include/spinnaker_driver/spinnaker_driver.hpp
    HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../spinnaker_driver
)

find_library(SPINNAKER_DRIVER_LIBRARY
    NAMES spinnaker_driver
    HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../spinnaker_driver
    PATH_SUFFIXES lib bin build
)

find_package_handle_standard_args(
    spinnaker_driver
    DEFAULT_MSG
    SPINNAKER_DRIVER_INCLUDE_DIR
    SPINNAKER_DRIVER_LIBRARY
)

if (SPINNAKER_DRIVER_FOUND)
    message(STATUS "spinnaker_driver include: ${SPINNAKER_DRIVER_INCLUDE_DIR}")
    message(STATUS "spinnaker_driver library: ${SPINNAKER_DRIVER_LIBRARY}")
endif()





