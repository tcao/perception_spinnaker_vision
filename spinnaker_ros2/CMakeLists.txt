cmake_minimum_required(VERSION 3.5)
project(spinnaker_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

if(ENABLE_SENSOR_TOPIC)
  add_compile_options(-DENABLE_SENSOR_TOPIC=1)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(spinnaker_driver REQUIRED)
find_package(spinnaker_perception_apps REQUIRED)
find_package(OpenCV REQUIRED)

# In my Humble/Ubuntu 22.0 box, colcon won't pass --cmake-args -DENABLE_APRILTAG=ON to cmake
# so in Linux build default ENABLE_APRILTAG to true
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(ENABLE_APRILTAG TRUE)
endif()
if(ENABLE_APRILTAG)
  add_compile_options(-DENABLE_APRILTAG=1)
  find_package(apriltag REQUIRED)
  # It seems Linux doesn't like add_link_options while Apple does
  # So for Linux, adding target_link_libraries as below
  if(APPLE)
    add_link_options(-L/usr/local/lib -lapriltag)
  endif()
endif()

include_directories(include)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp
  src/chessboard_pose_estimate.cpp
  src/apriltag_pose_estimate.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

# Can't not use if (UNIX), it applies to both Linux and macOS
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
target_link_libraries(${PROJECT_NAME}
  ${spinnaker_driver_TARGETS}
  apriltag
  ${OpenCV_LIBS})
endif()
ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "spinnaker_driver"
  "spinnaker_perception_apps"
  "OpenCV"
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
