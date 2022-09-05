cmake_minimum_required(VERSION 3.5)
project(spinnaker_driver VERSION 0.1.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # turn off unknown-pragmas warning
  add_compile_options(-Wno-unknown-pragmas)
endif()

# find dependencies
# the spinnaker SDK does not provide a cmake file
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Spinnaker REQUIRED)
find_package(OpenCV REQUIRED)
# Skip streaming app by changing the following to set(STREAMING FALSE)
set(STREAMING TRUE)
if(STREAMING)
find_package(GStreamer 1.0 REQUIRED COMPONENTS
  app
  video
)
endif()
find_package(ament_cmake REQUIRED)

include_directories(${SPINNAKER_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_library(${PROJECT_NAME}
  src/spinnaker_driver.cpp
)

add_executable(gige_acquisition apps/gigev_acquisition.cpp)
target_link_libraries(gige_acquisition
  ${PROJECT_NAME}
  ${SPINNAKER_LIBRARY}
  pthread
  ${OpenCV_LIBS})

if(STREAMING)
add_executable(machine_vision_streaming apps/machine_vision_streaming.cpp)
target_include_directories(machine_vision_streaming PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_include_directories(machine_vision_streaming PRIVATE ${GSTREAMER_APP_INCLUDE_DIRS})
target_include_directories(machine_vision_streaming PRIVATE ${OpenCV_DIRS})
if(APPLE)
  # force to point to libglib-2.0
  set(CMAKE_EXE_LINKER_FLAGS "-L/usr/local/lib")
endif()
target_link_libraries(machine_vision_streaming
  ${PROJECT_NAME}
  ${SPINNAKER_LIBRARY}
  pthread
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${GSTREAMER_AUDIO_LIBRARIES}
  ${GSTREAMER_VIDEO_LIBRARIES}
  glib-2.0
  gobject-2.0
  ${OpenCV_LIBS})
endif()

ament_package()
