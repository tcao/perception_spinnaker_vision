#
#  Find the Spinnaker SDK
#  FLIR's Spinnaker SDK build doesn't use cmake, so have to manually find it
#  By Ting Cao
#
#  Spinnaker_FOUND        - True if Spinnaker was found.
#  Spinnaker_LIBRARIES    - The libraries needed to use Spinnaker
#  Spinnaker_INCLUDE_DIRS - Location of Spinnaker.h
#

include(FindPackageHandleStandardArgs)

set(SPINNAKER_DIR "SPINNAKER_DIR" CACHE PATH "SPINNAKER_DIR")

find_path(SPINNAKER_INCLUDE_DIR
    NAMES Spinnaker.h
    PATHS
    # potential macOS paths
    /usr/include/spinnaker
    /usr/local/include/spinnaker
    /opt/spinnaker/include
    )

find_library(SPINNAKER_LIBRARY
    NAMES Spinnaker
    PATHS
    # potential macOS paths
    /usr/lib
    /usr/local/lib
    # known Ubuntu path
    /opt/spinnaker/lib
    )

find_package_handle_standard_args(
    Spinnaker
    DEFAULT_MSG
    SPINNAKER_INCLUDE_DIR
    SPINNAKER_LIBRARY
)

if (SPINNAKER_FOUND)
    message(STATUS "Spinnaker include: ${SPINNAKER_INCLUDE_DIR}")
    message(STATUS "Spinnaker library: ${SPINNAKER_LIBRARY}")
    set(SPINNAKER_INCLUDE_DIRS "${SPINNAKER_INCLUDE_DIR}")
    set(SPINNAKER_LIBRARIES "${SPINNAKER_LIBRARY}")
endif()
