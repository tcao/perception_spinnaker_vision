cmake_minimum_required(VERSION 3.5)
project(spinnaker_driver VERSION 1.0.0 LANGUAGES CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # turn off unknown-pragmas warning
  add_compile_options(-Wno-unknown-pragmas)
endif()

# find dependencies
# the spinnaker SDK does not provide a cmake file
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Spinnaker REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

include_directories(${SPINNAKER_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(${PROJECT_NAME}
  src/spinnaker_driver.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE Spinnaker::Spinnaker)

# associates an "export_*" target with the library target
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  #DIRECTORY include
  #DESTINATION ${CMAKE_INSTALL_PREFIX}
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Important to export Spinnaker symbols such that dependent packages have no need to explicitly link it
ament_export_targets(export_${PROJECT_NAME})
# Export include
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
