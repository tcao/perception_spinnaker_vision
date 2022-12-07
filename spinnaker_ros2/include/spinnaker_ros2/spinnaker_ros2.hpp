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

// Dependency spinnaker_perception_apps
#include "spinnaker_perception_apps/image_capture.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace spinnaker_ros2
{
using spinnaker_perception_apps::image_capture;
static const char defaultGigEVImageFrameID[] = "spinnaker_gigev";
static const char defaultUSB3ImageFrameID[] = "spinnaker_usb3";

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
    timestamp_(0)
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
  bool initialize();

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

protected:
  uint64_t timestamp_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};  // class SpinnakerRos2
}  // namespace spinnaker_ros2
#endif  // SPINNAKER_ROS2__SPINNAKER_ROS2_HPP_
