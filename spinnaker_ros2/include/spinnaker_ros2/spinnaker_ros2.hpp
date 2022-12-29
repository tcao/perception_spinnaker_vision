//
// Copyright 2022 Ting Cao <cao_ting@yahoo.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

///
/// ROS2 package for GigEV camera based on Spinnaker SDK
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

#ifndef SPINNAKER_ROS2__SPINNAKER_ROS2_HPP_
#define SPINNAKER_ROS2__SPINNAKER_ROS2_HPP_

// STD
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Dependency spinnaker_perception_apps
#include "spinnaker_perception_apps/image_capture.hpp"

// OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

namespace spinnaker_ros2
{
using spinnaker_perception_apps::image_capture;
static const char defaultGigEVImageFrameID[] = "spinnaker_gigev";
static const char defaultUSB3ImageFrameID[] = "spinnaker_usb3";

struct cli_parameters
{
  uint32_t camera;        // which camera to use, default should be provided
  float scale;
  std::string calib_xml;  // calibration xml
  /**
   * @brief Calibration grid pattern dimension
   * @remark As as frame rate, it is filled by driver's connect call
   */
  spinnaker_driver::ImageDimension dimension;
};

class SpinnakerRos2 : public rclcpp::Node, public image_capture
{
public:
  SpinnakerRos2(
    std::string node_name,
    rclcpp::NodeOptions node_options,
    std::string topic_name,
    void(*handler)(cv::Mat & input) = nullptr
  )
  : Node(node_name, node_options),
    image_capture(handler, 0, false),
    timestamp_(0),
    pose_detection_exit_(false),
    gui_timer_(nullptr),
    acquired_count_(0)
  {
    publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_name, rclcpp::SensorDataQoS());
  }
  /**
   * @brief Destroy the Spinnaker Ros 2 object
   * @remark Explicit destructor so shared_ptr has a opportunity to get taken care of
   */
  virtual ~SpinnakerRos2() {}

  /**
   * @brief Initialize SpinnakerRos2 instance after creation
   *
   * @return true
   * @return false
   */
  bool initialize(const cli_parameters & parameters);

  #pragma mark Overrides for image_capture
  void acquired(const spinnaker_driver::AcquiredImage * img) override;

  void image_release() override;

  static std::string & getEncoding(int cvType)
  {
    struct cvType2Encoding_t
    {
      int cvType;
      std::string encoding;
    };
    static cvType2Encoding_t cvType2Encodings[] = {
      {
        .cvType = CV_8UC1,
        .encoding = std::string("mono8"),
      },
      {
        .cvType = CV_8UC3,
        .encoding = std::string("bgr8"),
      },
      {
        .cvType = CV_16SC1,
        .encoding = std::string("mono16"),
      },
      {
        .cvType = CV_8UC4,
        .encoding = std::string("rgba8"),
      },
    };
    for (uint32_t i = 0; i < sizeof(cvType2Encodings) / sizeof(cvType2Encodings[0]); i++) {
      cvType2Encoding_t * entry = &cvType2Encodings[i];
      if (entry->cvType == cvType) {
        return entry->encoding;
      }
    }
    throw std::runtime_error("Unsupported encoding type");
  }

  void gui_timer_callback(void);

protected:
  /**
   * @brief Main thread for object detection and pose estimation
   */
  void pose_detect();

  /**
   * @brief Main thread to manage image processing,
   * including managing pose_detect thread
   */
  void image_process();

protected:
  uint64_t timestamp_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  /**
   * @brief Synchronization between camera driver envent handler and OpenCV rendering thread
   * @remark Used along with std::condition_variable frame_ready_condition_
   */
  std::mutex frame_ready_mutex_;

  /**
   * @brief Synchronization between camera driver envent handler and OpenCV rendering thread
   * @remark Used along with std::mutex frame_ready_mutex_
   */
  std::condition_variable frame_ready_condition_;

  /**
   * @brief Synchronization to access cv::
   */
  std::mutex critical_section_;

  volatile bool pose_detection_exit_;

  rclcpp::TimerBase::SharedPtr gui_timer_;
  /**
   * @brief Successful image acquisition count
   *
   */
  volatile uint32_t acquired_count_;

  // OpenCV related
  float scale_;
  cv::Size grid_size_;
  cv::Mat camera_intrinsic_;
  cv::Mat camera_distortion_;
  /**
   * @brief 3D grid pattern (chessboard) corners scene with origin at the center
   */
  std::vector<cv::Point3f> target_corners_;
  std::thread pose_detect_thread_;
};  // class SpinnakerRos2
}  // namespace spinnaker_ros2
#endif  // SPINNAKER_ROS2__SPINNAKER_ROS2_HPP_
