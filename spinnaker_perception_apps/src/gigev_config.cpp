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
/// GigEV camera acquisition application base on FLIR driver
/// @remark Image acquisition application with FLIR GigEV camera
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

// std headers
#include <string>
#include <vector>
#include <memory>

#include "spinnaker_perception_apps/image_capture.hpp"

namespace
{
using spinnaker_perception_apps::image_capture;
}  // anonymous namespace

void spin(spinnaker_driver::SpinnakerDriverGigE * gigev)
{
  using namespace std::chrono_literals;
  static const std::chrono::seconds kRunningThreadTimeout(10);

  std::this_thread::sleep_for(kRunningThreadTimeout);

  std::cout << "exiting from thread" << std::endl << std::flush;
  gigev->stop();
}

struct cli_parameters
{
  bool list_camera;     // option to list cameras
  uint32_t camera;      // which camera to use, default should be provided
  int32_t video_mode;   // optional/set - defaul to negative to indicate it is not set
  float frame_rate;     // optional/set - defaul to negative
  spinnaker_driver::ImageDimension dimension;   // optional for setting, - default to zero
};

const int32_t CLI_DEFAULT_VIDEO_MODE(-1);
const float CLI_DEFAULT_FRAME_RATE(0.0f);
const uint32_t CLI_DEFAULT_CAMERA(0);
const uint32_t CLI_DEFAULT_WIDTH(0);
const uint32_t CLI_DEFAULT_HEIGHT(0);
/**
 * @brief Parsing CLI parameters
 *
 * @param argc Count of parameters, including arg name and value pairs
 * @param argv Input paramters
 * @param parameters Output parameters
 * @param set_options - when set, one or more options is requested
 * @return true when All good
 * @return false when bad or help so exit
 */
bool argument_parse(int argc, char * argv[], cli_parameters * parameters, bool & set_options)
{
  bool help = false;
  // reset list option
  parameters->list_camera = false;
  bool mode_set = false,
    width_set = false,
    height_set = false,
    frame_rate_set = false;

  if (1 != argc) {
    // arguments parsing loop
    std::vector<std::string> args(argv + 1, argv + argc);
    for (auto i = args.begin(); i != args.end(); i++) {
      bool parsed = false;
      if ("--help" == *i) {
        help = parsed = true;
        break;
      }
      // List camera, no further argument is expected, all other options are ignored
      if ("-l" == *i) {
        parameters->list_camera = true;
        parsed = true;
        break;
      }

      // Image width, one more argument as value is expected
      if ("-w" == *i) {
        try {
          uint32_t width = (uint32_t)std::stoul(*++i);
          parameters->dimension.width = width;
          set_options = width_set = parsed = true;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad width argument, please rerun or current image width will be used" << std::endl;
          help = true;
          break;
        }
      }

      // Image height, one more argument as value is expected
      if ("-h" == *i) {
        try {
          uint32_t height = (uint32_t)std::stoul(*++i);
          parameters->dimension.height = height;
          set_options = height_set = parsed = true;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad height argument, please rerun or current image heigh will be used" << std::endl;
          help = true;
          break;
        }
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

      // Frame rate, one more argument as value is expected
      if ("-f" == *i) {
        try {
          float what = std::stof(*++i);
          parameters->frame_rate = what;
          set_options = frame_rate_set = parsed = true;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad frame rate argument, please rerun or current frame rate will be used" << std::endl;
          help = true;
          break;
        }
      }

      // Video mode
      if ("-m" == *i) {
        try {
          uint32_t which = (uint32_t)std::stoul(*++i);
          parameters->video_mode = which;
          parsed = true;
          set_options = mode_set = true;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad video mode argument, please rerun or current video mode will be used" << std::endl;
          help = true;
          break;
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
    std::cout << argv[0] << " [-l (list cameras)][-c <camera>][-f <frame rate>]" <<
      "[-m <video mode>][-h <image height>][-w <image width>][--help]" << std::endl;
    std::cout << "-l (optional) to list cameras and their important properties" << std::endl <<
      "   Some properties could be used other applications, say -m, -f, -h, -w for video mode" <<
      std::endl << "   It may needs to run multiple times to list all cameras" <<
      std::endl << "   All other options are ignored" << std::endl;

    std::cout << "-c (default=" << parameters->camera << ") to specify which camera to use" <<
      std::endl;
    std::cout << "   Optional when used with other options" << std::endl;
    std::cout << "   Required when used for image acquisition, " <<
      "ie., when no other options" << std::endl;

    std::cout << "-f (optional/setup only, default=" << parameters->frame_rate << ")" <<
      " to set frame rate" << std::endl <<
      "   If specified value exceeds the maximum allowed frame rate, the maximum is used" <<
      std::endl;

    std::cout << "-m (optional/setup only): to set video mode" << std::endl;
    std::cout << "   Please use the numeric values of video mode, listed from -l" << std::endl;

    std::cout << "-h (optional/setup only): to set image height in pixels" << std::endl;
    std::cout << "-w (optional/setup only): to set image width in pixels" << std::endl;

    std::cout << "Note: setup only options can be combined. After options are setup, " <<
      "rerun with -l to confirm" << std::endl;

    std::cout << "When no configuration parameter(s) is/are specified," <<
      "it runs for 10s to acquire images" << std::endl;

    // Don't continue after exit from this function
    return false;
  } else {
    if (!parameters->list_camera) {
      std::cout << "Camera be used: " << parameters->camera << std::endl;
    }
    // Show specified setting options
    if (set_options) {
      if (frame_rate_set) {
        std::cout << "Frame rate specified: " << parameters->frame_rate << std::endl;
      }

      if (width_set || height_set) {
        if (width_set && height_set) {
          std::cout << "Image dimension specified: " << parameters->dimension.width << "/" <<
            parameters->dimension.height << std::endl;
        } else {
          if (width_set) {
            std::cout << "Image width specified: " << parameters->dimension.width << std::endl;
          } else {
            std::cout << "Image height specified: " << parameters->dimension.height << std::endl;
          }
        }
      }

      if (mode_set) {
        std::cout << "Video mode specified: " << parameters->video_mode << std::endl;
      }
    }
  }
  return true;
}

int main(int argc, char * argv[])
{
  static cli_parameters parameters = {
    .list_camera = false,   // Not list
    .camera = CLI_DEFAULT_CAMERA,
    .video_mode = CLI_DEFAULT_VIDEO_MODE,
    .frame_rate = CLI_DEFAULT_FRAME_RATE,
    .dimension = {
      .width = CLI_DEFAULT_WIDTH,
      .height = CLI_DEFAULT_HEIGHT,
    },
  };

  bool set_options = false;
  if (!argument_parse(argc, argv, &parameters, set_options)) {
    return 0;
  }

  image_capture capture(
    [](cv::Mat & input) {
      static uint32_t count = 0;
      std::string fname = "gigev_config_";
      fname += std::to_string(count) + ".png";
      // shrink it to a quarter of the original when its width is beyoound 2000
      if (2000 < input.cols) {
        cv::Mat adjusted = cv::Mat();
        cv::resize(input, adjusted, cv::Size(), 0.5, 0.5);
        cv::imwrite(fname, adjusted);
      } else {
        cv::imwrite(fname, input);
      }
      count++;
    },
    static_cast<uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count()),
    true
  );
  spinnaker_driver::AcquiredImageCallback acquisition_callback =
    std::bind(&image_capture::acquired, &capture, std::placeholders::_1);

  bool continue_status = true;
  // Instantiate GigEV driver
  spinnaker_driver::SpinnakerDriverGigE gigev_driver;

  // List camera option
  if (parameters.list_camera) {
    if (!gigev_driver.list(spinnaker_driver::SupportedCameraType_e::CAMERA_TYPE_GIGE)) {
      std::cerr << "Failed to list GigEV cameras" << std::endl;
    }
    // Successful or not, no other options allow to run
    continue_status = false;
  }

  // Configurable paramters setting operation
  if (set_options) {
    std::vector<spinnaker_driver::ConfigurableParameter> config;
    if (CLI_DEFAULT_VIDEO_MODE != parameters.video_mode) {
      spinnaker_driver::ConfigurableParameter video_mode = {
        .type = spinnaker_driver::ConfigurableParameter_e::CONFIG_VIDEO_MODE,
        .data = static_cast<void *>(&parameters.video_mode),
      };
      config.push_back(video_mode);
    }
    if (CLI_DEFAULT_FRAME_RATE != parameters.frame_rate) {
      spinnaker_driver::ConfigurableParameter frame_rate = {
        .type = spinnaker_driver::ConfigurableParameter_e::CONFIG_FRAME_RATE,
        .data = static_cast<void *>(&parameters.frame_rate),
      };
      config.push_back(frame_rate);
    }
    if (CLI_DEFAULT_WIDTH != parameters.dimension.width) {
      spinnaker_driver::ConfigurableParameter width = {
        .type = spinnaker_driver::ConfigurableParameter_e::CONFIG_IMAGE_WIDTH,
        .data = static_cast<void *>(&parameters.dimension.width),
      };
      config.push_back(width);
    }
    if (CLI_DEFAULT_HEIGHT != parameters.dimension.height) {
      spinnaker_driver::ConfigurableParameter height = {
        .type = spinnaker_driver::ConfigurableParameter_e::CONFIG_IMAGE_HEIGHT,
        .data = static_cast<void *>(&parameters.dimension.height),
      };
      config.push_back(height);
    }

    gigev_driver.set_configurable_parameters(parameters.camera, config);
    config.clear();
    // Successful or not, no other options allow to run
    continue_status = false;
  }

  // Normal acquisition operation
  if (continue_status) {
    std::cout << "Connecting to camera: " << parameters.camera << std::endl;
    if (gigev_driver.connect(parameters.camera)) {
      // start a process running for 10s
      std::shared_ptr<std::thread> spin_thread = std::make_shared<std::thread>(
        &spin, &gigev_driver);
      bool thread_status = gigev_driver.start(
        acquisition_callback, parameters.frame_rate);
      if (thread_status) {
        std::cout << "Acquistion is successfully terminated" << std::endl;
      } else {
        std::cerr << "Failed to start camera: " << parameters.camera << std::endl;
      }
      if (spin_thread->joinable()) {
        spin_thread->join();
        std::cout << "User spin thread terminated-0" << std::endl;
      }
      std::cout << "User spin thread terminated-1" << std::endl << std::flush;
    } else {
      std::cerr << "Failed to connect camera: " << parameters.camera << std::endl;
    }
  }

  try {
    gigev_driver.release();
  } catch (const std::exception & e) {
    std::cerr << "User forced gigev_driver.release exception: " << e.what() << '\n';
  }

  std::cout << std::flush;
  return 0;
}
