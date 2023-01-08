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
  uint32_t camera_;         // which camera to use, default should be provided
  float scale_;
  std::string calib_xml_;   // calibration xml
  /**
   * @brief Calibration grid pattern dimension
   * @remark As as frame rate, it is filled by driver's connect call
   */
  spinnaker_driver::ImageDimension dimension_;
};

/**
 * @brief Interface defined for image processing synchronization
 */
class ProcessingSync
{
public:
  /**
   * @brief Image manipuation critical section enter/exit
   * @param enter - Enter critical section when true, exit when false
   * @remark THis is a blocking call
   */
  virtual void enter_critical_section(bool enter) = 0;
  virtual uint32_t get_sequence_number() = 0;
  virtual bool is_sequence_number_changed(uint32_t previous) = 0;
  virtual void get_camera_parameters(
    cv::Mat & camera_intrinsic, cv::Mat & camera_distortion) const = 0;
  /**
   * @brief Submit composed image for rendering
   */
  virtual void submit() = 0;
  /**
   * @brief Get the image object
   *
   * @return const cv::Mat
   */
  virtual const cv::Mat get_image() const = 0;
};

typedef std::shared_ptr<ProcessingSync> ProcessingSyncPtr;

// Forward declaration
class SpinnakerRos2;

class ImageProcessInstance
{
  struct image_process_context
  {
    std::mutex processing_ready_mutex_;
    std::condition_variable processing_ready_condition_;
    volatile bool ready_;
    bool processing_exit_;
  };

public:
  explicit ImageProcessInstance(ProcessingSyncPtr sync)
  : sync_(sync)
  {}
  virtual ~ImageProcessInstance() {}
  virtual void ready() = 0;
  virtual void process() = 0;

protected:
  ProcessingSyncPtr sync_;
  image_process_context image_process_context_;
};
typedef std::shared_ptr<ImageProcessInstance> ImageProcessInstancePtr;

class SpinnakerRos2 : public rclcpp::Node, public image_capture, public ProcessingSync
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
    image_process_exit_(false),
    gui_timer_(nullptr),
    acquired_count_(0)
  {
    // Personally prefer to use SensorDataQoS, but to be compatible with existing tool, such as
    // image_tools package from https://github.com/ros2/demos, use its default one
    // QoSInitialization(rmw_qos_history_policy_t history_policy_arg, size_t depth_arg);

    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        // Depth represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        10
    ));
    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      topic_name, qos /*rclcpp::SensorDataQoS()*/);
  }
  /**
   * @brief Destroy the Spinnaker Ros 2 object
   * @remark Explicit destructor so shared_ptr has a opportunity to get taken care of
   */
  virtual ~SpinnakerRos2() {}

  /**
   * @brief Add an image process instance
   * @remark Call this before initialize
   * @param instance
   */
  void add_image_process_instance(ImageProcessInstancePtr instance);

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

  #pragma mark Overrides for ProcessingSync
  void enter_critical_section(bool enter) override;
  uint32_t get_sequence_number() override;
  bool is_sequence_number_changed(uint32_t previous) override;
  void get_camera_parameters(
    cv::Mat & camera_intrinsic, cv::Mat & camera_distortion) const override;
  const cv::Mat get_image() const override;
  void submit() override;

protected:
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
   * @brief Synchronization to access cv::Mat _image of base class image_capture
   */
  std::mutex critical_section_;

  // lock_guard and unique_lock uses RAII,
  // so we initialize the lock heare w/o actually lock it by specifying std::defer_lock
  std::unique_lock<std::mutex> critical_section_lock_{critical_section_, std::defer_lock};

  volatile bool image_process_exit_;

  rclcpp::TimerBase::SharedPtr gui_timer_;
  /**
   * @brief Successful image acquisition count
   */
  volatile uint32_t acquired_count_;

  // OpenCV related
  cv::Mat camera_intrinsic_;
  cv::Mat camera_distortion_;

  std::thread pose_detect_thread_;

  /**
   * @brief Image processing context stack
   */
  std::vector<ImageProcessInstancePtr> image_process_instances_;
};  // class SpinnakerRos2
}  // namespace spinnaker_ros2
#endif  // SPINNAKER_ROS2__SPINNAKER_ROS2_HPP_
