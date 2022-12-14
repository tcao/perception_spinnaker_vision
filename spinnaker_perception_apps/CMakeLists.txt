cmake_minimum_required(VERSION 3.5)
project(spinnaker_perception_apps VERSION 1.0.0 LANGUAGES CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # turn off unknown-pragmas warning
  add_compile_options(-Wno-unknown-pragmas)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(spinnaker_driver REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

#find_package(ament_cmake_auto REQUIRED)

# the spinnaker SDK does not provide a cmake file
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(PERCEPTION_APP_DEPENDENCIES
  "spinnaker_driver"
  "OpenCV"
  "Threads"
)
# Skip streaming app by changing the following to set(STREAMING FALSE)
set(STREAMING TRUE)
if(STREAMING)
find_package(GStreamer 1.0 REQUIRED COMPONENTS
  app
  video
)
set(PERCEPTION_APP_DEPENDENCIES "${PERCEPTION_APP_DEPENDENCIES}" "GStreamer")
endif()

#foreach(pkg ${PERCEPTION_APP_DEPENDENCIES})
#  find_package(${pkg} REQUIRED)
#endforeach()

#ament_auto_find_build_dependencies(REQUIRED ${PERCEPTION_APP_DEPENDENCIES})

#include_directories(include)
include_directories(${spinnaker_driver_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set(GIGEV_CONFIG "gigev_config")
add_executable(${GIGEV_CONFIG}
  src/${GIGEV_CONFIG}.cpp
)
target_include_directories(${GIGEV_CONFIG} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_DIRS})
# ament_auto_find_build_dependencies/ament_auto_add_executable having trouble with pthread
# Have to manually help it with link
target_link_libraries(${GIGEV_CONFIG}
  #${SPINNAKER_DRIVER_LIBRARY}
  ${spinnaker_driver_TARGETS}
  Threads::Threads
  ${OpenCV_LIBS})

if(STREAMING)
set(MACHINE_VISION_STREAMING "machine_vision_streaming")
add_executable(${MACHINE_VISION_STREAMING}
  src/${MACHINE_VISION_STREAMING}.cpp
)
# ament_auto_find_build_dependencies/ament_auto_add_executable having trouble with GStreamer
# Have to manually help it with compile
target_include_directories(${MACHINE_VISION_STREAMING} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${OpenCV_DIRS})
if(APPLE)
  # force to point to libglib-2.0
  set(CMAKE_EXE_LINKER_FLAGS "-L/usr/local/lib")
endif()
# ament_auto_find_build_dependencies/ament_auto_add_executable having trouble with GStreamer
# Have to manually help it with link
target_link_libraries(${MACHINE_VISION_STREAMING}
  ${spinnaker_driver_TARGETS}
  Threads::Threads
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${GSTREAMER_AUDIO_LIBRARIES}
  ${GSTREAMER_VIDEO_LIBRARIES}
  glib-2.0
  gobject-2.0
  ${OpenCV_LIBS})
endif()

install(TARGETS
  ${GIGEV_CONFIG}
  DESTINATION lib/${PROJECT_NAME}/)

# install app's include so it can be used to subclass
install(
    DIRECTORY include
    DESTINATION ${CMAKE_INSTALL_PREFIX})

if(STREAMING)
  install(TARGETS
    ${MACHINE_VISION_STREAMING}
    DESTINATION lib/${PROJECT_NAME}/)
endif()

# Export include to enable subclass
ament_export_include_directories(include)

ament_package()
