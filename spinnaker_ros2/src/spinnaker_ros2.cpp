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
/// Use the following to remap topic: --ros-args -r /spinnaker_ros2/image:=/image
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

// STD
#include <string>
#include <vector>
#include <memory>
#include <utility>

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

#include "spinnaker_ros2/spinnaker_ros2.hpp"

namespace
{
static const char defaultNodeName[] = "spinnaker_ros2";
static const char defaultImageTopicName[] = "image";
static const uint32_t CLI_DEFAULT_CAMERA(0);
static const rclcpp::Logger LOGGER = rclcpp::get_logger("spinnaker_ros2");
static const int32_t maxImageWidthNotResized(1200);
std::function<void(int)> shutdown_handler;

void signal_handler(int signal)
{
  shutdown_handler(signal);
}

}  // anonymous namespace

namespace spinnaker_ros2
{
bool SpinnakerRos2::initialize()
{
  return true;
}
#pragma mark Overrides for image_capture
void SpinnakerRos2::acquired(const spinnaker_driver::AcquiredImage * img)
{
  // count for normal operations
  static uint32_t count = 0;
  // count for errornous operations
  static uint32_t incomplete_count = 0;
  // count for unknown acquisition image format
  static uint32_t unspported_format_count = 0;

  if (nullptr != img->data_) {
    // TODO(tcao): Adding a lock if previous data is still been processing
    timestamp_ = img->timestamp_;
    // When Bayer pattern is used (as CCD/CMOS sensor usually do),
    // raw data is organized in single channel, such as Spinnaker GigEV camera Bayer based image
    // conversion from CV_8UC1 to CV_8UC3 is needed
    cv::Mat frame = cv::Mat(
      static_cast<int>(img->height_), static_cast<int>(img->width_),
      (3 == img->channels_) ? CV_8UC3 : CV_8UC1,
      // @remark No data is allocated, it just points to provided data
      const_cast<void *>(reinterpret_cast<const void *>(img->data_)),
      // TODO(tcao): Mono8 format has stride, but use or not doesn't seem matter
      img->stride_);

    // Convert to proper pixel format
    bool converted = false;
    switch (img->pixel_format_) {
      case spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_MONO8:
        converted = true;
        image_ = frame.clone();
        break;

      case spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_GB8:
      case spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_RG8:
        {
          cv::Mat frameBGR = cv::Mat(
            static_cast<int>(img->height_), static_cast<int>(img->width_), CV_8UC3);
          if (spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_GB8 == img->pixel_format_) {
            cv::cvtColor(frame, frameBGR, CV_BayerGR2BGR);
          } else {
            cv::cvtColor(frame, frameBGR, CV_BayerBG2BGR);
          }
          converted = true;
          image_ = frameBGR.clone();
        }
        break;

      case spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_BGR8:
        // frame is already in proper format, as in the above in case of PIXEL_FORMAT_MONO8
        converted = true;
        image_ = frame.clone();
        break;

      default:
        // converted won't be set for unsupported formats
        break;
    }
    if (converted) {
      RCLCPP_INFO(LOGGER, "acquired: %d", count++);
      if (image_handler) {
        // Call user ligic if it is installed
        image_handler(image_);
      }
      image_release();
    } else {
      std::cerr << "Unable to convert, please add the format (" << img->pixel_format_ <<
        ") to the support list" << std::endl;
      RCLCPP_ERROR(LOGGER, "acquired with unsupported format: %d", unspported_format_count++);
    }
  } else {
    // This could happen a lot in packet flooded networking environment, such as
    // 1 - ROS2 default DDS is multicast
    // 2 - GigEV at least heart beat is multicast
    // 3 - Some robots, such as Universal Robots, depends on networking for control and exchange
    RCLCPP_ERROR(LOGGER, "acquired with incomplete data: %d", incomplete_count++);
  }
}

void SpinnakerRos2::image_release()
{
  static uint32_t count = 0;
  uint64_t timestamp = timestamp_;
  // Create unique_ptr for Image message
  sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
  msg->header.stamp.sec = static_cast<int32_t>(timestamp / 1000000000);
  msg->header.stamp.nanosec = static_cast<uint32_t>(timestamp % 1000000000);
  msg->header.frame_id = defaultGigEVImageFrameID;
  msg->height = image_.rows;
  msg->width = image_.cols;
  try {
    msg->encoding = getEncoding(image_.type());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(LOGGER, "Unknown/Unsupported CV type: %d", image_.type());
    // Skip
    return;
  }
  msg->is_bigendian = false;
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(image_.step);
  msg->data.assign(image_.datastart, image_.dataend);
  publisher_->publish(std::move(msg));
  RCLCPP_INFO(LOGGER, "sensor_msgs::msg::Image published %d", count++);
}

}  //  namespace spinnaker_ros2

struct cli_parameters
{
  uint32_t camera;      // which camera to use, default should be provided
};


/**
 * @brief Parsing CLI parameters
 *
 * @param argc Count of parameters, including arg name and value pairs
 * @param argv Input paramters
 * @param parameters Output parameters
 * @return true when All good
 * @return false when bad or help so exit
 */
bool argument_parse(int argc, char * argv[], cli_parameters * parameters)
{
  bool help = false;

  if (1 != argc) {
    // arguments parsing loop
    std::vector<std::string> args(argv + 1, argv + argc);
    for (auto i = args.begin(); i != args.end(); i++) {
      bool parsed = false;
      if (("--help" == *i) || ("-h" == *i)) {
        help = parsed = true;
        break;
      }

      // Connect to specified camera, one more argument as value is expected
      if ("-c" == *i) {
        // Get camera index when specified
        try {
          uint32_t which = (uint32_t)std::stoul(*++i);
          parameters->camera = which;
          parsed = true;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad camera argument, use default: " << parameters->camera << std::endl;
        }
      }
    }
  }

  if (help) {
    std::cout << argv[0] << " [-c <camera>] [--help] [-h]" << std::endl;

    std::cout << "-c (default=" << parameters->camera << ") to specify which camera to use" <<
      std::endl;

    std::cout << "--help or -h showing this help" << std::endl;

    return false;
  }

  return true;
}

int main(int argc, char * argv[])
{
  static cli_parameters parameters = {
    .camera = CLI_DEFAULT_CAMERA,
  };

  if (!argument_parse(argc, argv, &parameters)) {
    return 0;
  }

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  std::string topic(defaultNodeName);
  topic += "/";
  topic += defaultImageTopicName;
  auto const spinnaker_ros2 = std::make_shared<spinnaker_ros2::SpinnakerRos2>(
    defaultNodeName,
    rclcpp::NodeOptions().use_intra_process_comms(true),
    topic,
    // The whole purpose this user provide callback is to scale down the image when necessary.
    // But anything could be done before it is published
    [](cv::Mat & input) {
      if (maxImageWidthNotResized < input.cols) {
        cv::resize(input, input, cv::Size(), 0.5, 0.5);
      }
    }
  );

  executor.add_node(spinnaker_ros2);

  if (spinnaker_ros2->initialize()) {
    // Instantiate GigEV driver
    spinnaker_driver::SpinnakerDriverGigE gigev_driver;
    std::cout << "Connecting to camera: " << parameters.camera << std::endl;
    if (gigev_driver.connect(parameters.camera)) {
      // Install user break handler
      // @remark see url why it is done this way:
      // https://stackoverflow.com/questions/11468414/using-auto-and-lambda-to-handle-signal
      bool signaled = false;
      std::signal(SIGINT, signal_handler);
      std::signal(SIGTERM, signal_handler);
      shutdown_handler = [&](int signal) {
          std::cout << "signal handler called for: " << signal << " with " <<
          (signaled ? "signanled" : "!signaled") << std::endl;
          std::cout << std::flush;
          if (!signaled) {
            // Protect gigev_driver to make sure stop is only called once
            signaled = true;
            gigev_driver.stop();
            // Wait long enough for driver running loop to quit
            std::this_thread::sleep_for(2 * spinnaker_driver::kRunningThreadWakeup);
          }
          rclcpp::shutdown();
        };

      spinnaker_driver::AcquiredImageCallback acquisition_callback =
        std::bind(&spinnaker_ros2::SpinnakerRos2::acquired, spinnaker_ros2, std::placeholders::_1);
      // Running image acquistion in a detached thread such that we can spin ROS
      if (gigev_driver.start(acquisition_callback, true)) {
        std::cout << "gigev_driver starts" << std::endl;
        std::cout << std::flush;
        executor.spin();
      } else {
        std::cerr << "gigev_driver fails to start" << std::endl;
      }
    } else {
      std::cerr << "gigev_driver fails to connect" << std::endl;
    }

    try {
      // Since driver runs in a detached state,
      // let's wait till it's done with long time deinit sequence
      std::cout << "Waiting driver deinit to finish..." << std::endl;
      while (!gigev_driver.is_deinit_done()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      gigev_driver.release();
    } catch (const std::exception & e) {
      std::cerr << "gigev_driver.release exception: " << e.what() << '\n';
    }
  }

  rclcpp::shutdown();

  std::cout << std::flush;

  return 0;
}
