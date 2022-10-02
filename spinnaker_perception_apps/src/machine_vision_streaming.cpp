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

///
/// GigEV machine vision streaming application based on spinnaker_driver
/// @remark Machine vision streaming application with FLIR GigEV camera
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

// std headers
#include <memory>
#include <string>
#include <vector>

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

#include "spinnaker_perception_apps/vision_streaming.hpp"
// Local CV algorithms
#include "spinnaker_perception_apps/edgedetector.h"

namespace
{
using spinnaker_perception_apps::image_capture;
using spinnaker_perception_apps::VisionStream;
}  // anonymous namespace

#pragma mark - Main application vision processing data and setup
struct filter_threshold
{
  double low;
  double high;
};

struct filter_operation
{
  filter_threshold threshold;
  bool inverse;
  std::string name;
};

struct filter_operation_set
{
  filter_operation * operation;
  void ( * filter)(const cv::Mat & input, filter_operation * operation_data, cv::Mat & output);
};

EdgeDetector edge_detector;
void sobel_filter(const cv::Mat & input, filter_operation * operation_data, cv::Mat & output)
{
  edge_detector.computeSobel(input);
  std::cout << "filter: " << operation_data->name << std::endl;
  if (!operation_data->inverse) {
    output = edge_detector.getBinaryMap(operation_data->threshold.low);
    return;
  }

  output = 1.0 - edge_detector.getBinaryMap(operation_data->threshold.low);
}

void canny_filter(const cv::Mat & input, filter_operation * operation_data, cv::Mat & output)
{
  std::cout << "filter: " << operation_data->name << std::endl;
  cv::Canny(input, output, operation_data->threshold.low, operation_data->threshold.high);
  if (!operation_data->inverse) {
    output = 255 - output;
  }
}

void no_filter(const cv::Mat & input, filter_operation * operation_data, cv::Mat & output)
{
  std::cout << "filter: " << operation_data->name << std::endl;
  if (!operation_data->inverse) {
    output = input.clone();
    return;
  }
  cv::bitwise_not(input, output);
}

enum FilterOperation_e
{
  FILTER_OPERATION_NONE,
  FILTER_OPERATION_NONE_INVERSE,
  FILTER_OPERATION_CANNY,
  FILTER_OPERATION_CANNY_INVERSE,
  FILTER_OPERATION_SOBEL_LOW,
  FILTER_OPERATION_SOBEL_LOW_INVERSE,
  FILTER_OPERATION_SOBEL_HIGH,
  FILTER_OPERATION_SOBEL_HIGH_INVERSE,

  FILTER_OPERATION_NUM,
};

filter_operation filter_operations[FILTER_OPERATION_NUM] = {
  {
    // 0 - raw input, ie, no filter applied, threshold is not used
    .threshold = {
      .low = 0.0,
      .high = 0.0,
    },
    .inverse = false,
    .name = "Input/Regular",
  },
  {
    // 1 - inverse raw input, no filter applied, but inversed
    .threshold = {
      .low = 0.0,
      .high = 0.0,
    },
    .inverse = true,
    .name = "Input/Inverse",
  },
  {
    // 2 - Canny
    .threshold = {
      .low = 125.0,
      .high = 350.0,
    },
    .inverse = false,
    .name = "Canny",
  },
  {
    // 3 - Canny/inverse
    .threshold = {
      .low = 125.0,
      .high = 350.0,
    },
    .inverse = true,
    .name = "Canny/Inverse",
  },
  {
    // 4 - Sobel low threshold
    .threshold = {
      .low = 125.0,
      .high = 0.0,  // Not used
    },
    .inverse = false,
    .name = "Sobel/Low Threshold",
  },
  {
    // 5 - Sobel low threshold/inverse
    .threshold = {
      .low = 125.0,
      .high = 0.0,  // Not used
    },
    .inverse = true,
    .name = "Sobel/Low Threshold/Inverse",
  },
  {
    // 6 - Sobel hight threshold
    .threshold = {
      .low = 350.0,
      .high = 0.0,  // Not used
    },
    .inverse = false,
    .name = "Sobel/High Threshold",
  },
  {
    // 7 - Sobel hight threshold/inverse
    .threshold = {
      .low = 350.0,
      .high = 0.0,  // Not used
    },
    .inverse = true,
    .name = "Sobel/High Threshold/Inverse",
  },
};

filter_operation_set filter_operation_sets[] = {
  {
    .operation = &filter_operations[FILTER_OPERATION_NONE],
    .filter = no_filter,
  },
  {
    .operation = &filter_operations[FILTER_OPERATION_NONE_INVERSE],
    .filter = no_filter,
  },
  {
    .operation = &filter_operations[FILTER_OPERATION_CANNY],
    .filter = canny_filter,
  },
  {
    .operation = &filter_operations[FILTER_OPERATION_CANNY_INVERSE],
    .filter = canny_filter,
  },
  {
    .operation = &filter_operations[FILTER_OPERATION_SOBEL_LOW],
    .filter = sobel_filter,
  },
  {
    .operation = &filter_operations[FILTER_OPERATION_SOBEL_LOW_INVERSE],
    .filter = sobel_filter,
  },
  {
    .operation = &filter_operations[FILTER_OPERATION_SOBEL_HIGH],
    .filter = sobel_filter,
  },
  {
    .operation = &filter_operations[FILTER_OPERATION_SOBEL_HIGH_INVERSE],
    .filter = sobel_filter,
  },
};

#pragma mark - Main application pipeline data and setup
struct cli_parameters
{
  /**
   * @brief Host IP address streaming to
   */
  std::string ip_address;
  /**
   * @brief First IP port of of the host
   * @remark If multiple ports are used, next port is current one plus one
   */
  uint16_t first_port;
  /**
   * @brief frame_rate
   * @remark Currently is not settable, ie, it's using the current frame rate
   * @remark It is filled by driver's connect call
   */
  float frame_rate;
  /**
   * @brief Which camera to use
   */
  uint32_t camera;

  /**
   * @brief Image dimension
   * @remark As as frame rate, it is filled by driver's connect call
   */
  spinnaker_driver::ImageDimension dimension;
};

/**
 * @brief Set the up pipelines object
 *
 * @param vision_stream
 * @param parameters
 */
void setup_pipelines(std::shared_ptr<VisionStream> vision_stream, cli_parameters * parameters)
{
  const uint32_t WHEN_TO_SAVE_DEBUG_IMAGE(62);  // Save input images in this successful capture
  const uint32_t EACH_FILTER_RUNS(30);          // Each filter runs this many times

  uint32_t udp_port = parameters->first_port;
  static VisionStream::VisionStreamProperty streams[] = {
    // Raw input image pipeline, assume it is in BGR default OpenCV format
    {
      .width = parameters->dimension.width,
      .height = parameters->dimension.height,
      // Default OpenCV format
      .pixel_format = "BGR",
      .client_ip = parameters->ip_address,
      // Assume next pipeline's port is increased by one
      .client_port = udp_port,
      .frame_rate = parameters->frame_rate,
      .timestamp = 0,
      .loop = nullptr,
      .appsrc = nullptr,
      .appsrc_logic = [](const cv::Mat & input, cv::Mat & output) -> bool {
          // Not doing anything, just pass along w/o affecting passing input
          static uint32_t count = 0;
          if ((1 == input.channels()) && (CV_8U == input.type())) {
            // Streaming expects color image but this is a gray one, so convert
            cv::cvtColor(input, output, CV_GRAY2BGR);
          } else {
            // Clone to output w/o touching input
            output = input.clone();
          }
          if (WHEN_TO_SAVE_DEBUG_IMAGE == count) {
            std::cout << "BGR - saving image in the format of: " << input.type() << std::endl;
            cv::imwrite("mv_bgr.png", output);
          }
          count++;
          return true;
        },
    },
    // Computer vision processing pipeline, assuming it is in GRAY (single channel) format
    {
      .width = parameters->dimension.width,
      .height = parameters->dimension.height,
      // Gray
      .pixel_format = "GRAY8",
      .client_ip = parameters->ip_address,
      // Assume next pipeline's port is increased by one
      .client_port = udp_port + 1,
      .frame_rate = parameters->frame_rate,
      .timestamp = 0,
      .loop = nullptr,
      .appsrc = nullptr,
      .appsrc_logic = [](const cv::Mat & input, cv::Mat & output) -> bool {
          // Convert to GRAY if it is not already in the format
          static uint32_t count = 0;
          static uint32_t filter_iteration = 0;
          filter_operation_set * filter_operation = &filter_operation_sets
            [filter_iteration++ / EACH_FILTER_RUNS];

          // Wrap filter_iteration index
          if ((EACH_FILTER_RUNS * sizeof(filter_operation_sets) /
            sizeof(filter_operation_sets[0])) <=
            filter_iteration)
          {
            filter_iteration = 0;
          }
          if ((1 == input.channels()) && (CV_8U == input.type())) {
            if (WHEN_TO_SAVE_DEBUG_IMAGE == count) {
              std::cout << "GRAY8_0 - saving image in the format of: " << input.type() << std::endl;
              cv::imwrite("mv_gry.png", input);
            }
            count++;
            // No need to clone input, since it is already in proper format
            filter_operation->filter(input, filter_operation->operation, output);
          } else {
            cv::cvtColor(input, output, cv::COLOR_RGB2GRAY);
            if (WHEN_TO_SAVE_DEBUG_IMAGE == count) {
              std::cout << "GRAY8_1 - saving image in the format of: " << output.type() <<
                std::endl;
              cv::imwrite("mv_gry_1.png", output);
            }
            filter_operation->filter(output, filter_operation->operation, output);
            // Note filter may change output's pixel_format, let's address only CV_32F for now
            if (CV_32F == output.type()) {
              // Map 0.0 - 1.0 to 0 - 255
              output.convertTo(output, CV_8U, 255., 0.);
            }
            if (WHEN_TO_SAVE_DEBUG_IMAGE == count) {
              std::cout << "GRAY8_2 - saving image in the format of: " << output.type() <<
                std::endl;
              cv::imwrite("mv_gry_2.png", output);
            }
            count++;
          }
          return true;
        },
    },
  };

  vision_stream->register_streams(streams, sizeof(streams) / sizeof(streams[0]));
}

#pragma mark - Main application
const char CLI_DEFAULT_IP_ADDRESS[] = "127.0.0.1";
const uint16_t CLI_DEFAULT_FIRST_PORT = 5000;
const uint32_t CLI_DEFAULT_CAMERA = 0;

/**
 * @brief Parse and setup CLI parameters
 *
 * @param argc
 * @param argv
 * @param parameters
 * @return true
 * @return false Something may or may not be wrong, but don't continue
 * @remark TODO(tcao): Adding image width/height parsing
 */
bool argument_parse(int argc, char * argv[], cli_parameters * parameters)
{
  bool help = false;

  if (1 != argc) {
    std::vector<std::string> args(argv + 1, argv + argc);

    for (auto i = args.begin(); i != args.end(); i++) {
      bool parsed = false;
      if ("--help" == *i) {
        parsed = true;
        help = true;
        break;
      }
      // Connect to specified camera. One more argument is expected
      if ("-c" == *i) {
        // Get camera index when specified
        try {
          uint32_t which = (uint32_t)std::stoul(*++i);
          parameters->camera = which;
          std::cout << "Camera specified: " << which << std::endl;
          parsed = true;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad argument, use default camera: " << parameters->camera << std::endl;
        }
      }

      // Host IP address. One more argument is required
      if ("-s" == *i) {
        try {
          std::string ip = *++i;
          parameters->ip_address = ip;
          std::cout << "Host IP specified: " << ip << std::endl;
          parsed = true;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad argument, use default IP address: " << parameters->ip_address << std::endl;
        }
      }

      if (!parsed) {
        std::cerr << "Unknown argument skipped: " << *i << std::endl;
      }
    }
  } else {
    // No CLI argument specified, so show help
    help = true;
  }

  if (help) {
    std::cout << argv[0] << " [-c <camera>] [-s <host IP>] [--help]" << std::endl;
    std::cout << "-c (optional, default=" << parameters->camera <<
      ") to specify which camera to use" << std::endl;
    std::cout << "   Optional when used with other options" << std::endl;

    std::cout << "-s (optional, default=" << parameters->ip_address <<
      ") to specify host IP address streaming to" << std::endl;
    std::cout << "   Note default port starting from: " << parameters->first_port << std::endl;

    std::cout << "--help showing this help" << std::endl;

    return false;
  }
  return true;
}

int main(int argc, char * argv[])
{
  static cli_parameters parameters = {
    .ip_address = CLI_DEFAULT_IP_ADDRESS,
    .first_port = CLI_DEFAULT_FIRST_PORT,
    // Don't care about frame rate, it is filled by the system
    .frame_rate = 1.0f,
    .camera = CLI_DEFAULT_CAMERA,
    // Don't care about image size, it is filled by the system
    .dimension = {
      .width = 0,
      .height = 0,
    },
  };

  if (!argument_parse(argc, argv, &parameters)) {
    return 0;
  }

  std::shared_ptr<VisionStream> vision_stream = std::make_shared<VisionStream>();
  // The receiving pipeline doesn't need camera specific info
  vision_stream->receive();

  // Instantiate GigEV driver
  spinnaker_driver::CameraParameters camera_parameters;
  spinnaker_driver::SpinnakerDriverGigE gigev_driver;
  std::cout << "Connecting to camera " << parameters.camera << std::endl;
  // Connecting GigEV camera, with camera parameters requested
  if (gigev_driver.connect(
      parameters.camera,
      spinnaker_driver::SupportedCameraType_e::CAMERA_TYPE_GIGE,
      &camera_parameters))
  {
    // Specifying receiving thread callback
    spinnaker_driver::AcquiredImageCallback acquisition_callback =
      std::bind(&VisionStream::acquired, vision_stream, std::placeholders::_1);

    // Fill corresponding setup parameter with camera's current parameters
    parameters.dimension = camera_parameters.image_dimension;
    parameters.frame_rate = camera_parameters.frame_rate;

    setup_pipelines(vision_stream, &parameters);

    // Start streaming sending pipelines
    vision_stream->send();

    // This demonstration - exiting after specified seconds
    std::thread spin_thread([&gigev_driver, vision_stream]() {
        // Run the streaming for this amount of time in seconds
        std::this_thread::sleep_for(std::chrono::seconds(120));
        vision_stream->stop();
        gigev_driver.stop();
      });

    // Start image acquisition, no return till forced to
    bool thread_status = gigev_driver.start(acquisition_callback, false, parameters.frame_rate);
    if (thread_status) {
      std::cout << "Acquistion is successfully terminated" << std::endl;
    } else {
      std::cout << "Failed to start camera: " << parameters.camera << std::endl;
    }
    if (spin_thread.joinable()) {
      spin_thread.join();
      std::cout << "User spin thread terminated-0" << std::endl;
    }
    std::cout << "User spin thread terminated-1" << std::endl << std::flush;
  } else {
    std::cout << "Failed to connect camera: " << parameters.camera << std::endl;
  }

  try {
    gigev_driver.release();
  } catch (const std::exception & e) {
    std::cerr << "User forced gigev_driver.release exception: " << e.what() << '\n';
  }

  std::cout << std::flush;

  return 0;
}
