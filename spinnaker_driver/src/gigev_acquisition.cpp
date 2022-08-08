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
/// GigEV camera acquistion application base on FLIR driver
/// @remark Image acquistion application with FLIR GigEV camera
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <numeric>
#include <iomanip>
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "spinnaker_driver/spinnaker_driver.hpp"

class image_capture
{
public:
  image_capture()
  {
    auto now = std::chrono::system_clock::now();
    timestamp_ = now.time_since_epoch().count();
  }
  virtual ~image_capture() {}

  void acquired(const spinnaker_driver::AcquiredImage * img)
  {
    static uint32_t count = 0;
    std::chrono::system_clock::time_point current_timestamp{
      std::chrono::system_clock::duration{img->timestamp_}};
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      current_timestamp.time_since_epoch()) % 1000;
    const std::time_t t_c = std::chrono::system_clock::to_time_t(current_timestamp);
    std::cout << count << ": " << std::put_time(std::localtime(&t_c), "%F %T");
    std::cout << "." << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
    std::cout << "delta: " << img->timestamp_ - timestamp_ << " in ns" << std::endl;

    std::cout << "width: " << img->width_ << "/" << img->x_padding_ << std::endl;
    std::cout << "height: " << img->height_ << "/" << img->y_padding_ << std::endl;
    std::cout << "stride: " << img->stride_ << ", bpp: " << img->bits_per_pixel_ <<
      ", channels: " << img->channels_ << std::endl;
    if (nullptr != img->data_) {
      std::cout << "pixel format: " << (img->pixel_format_ & 0xff) << ", pixexl type: " <<
      ((img->pixel_format_ >> 8) & 0xff) << std::endl << std::flush;

      std::cout << "data: " << std::hex << img->data_ << std::endl << std::flush;
      std::cout << std::dec << std::endl;

      cv::Mat frame = cv::Mat(
        static_cast<int>(img->height_), static_cast<int>(img->width_),
        (3 == img->channels_) ? CV_8UC3 : CV_8UC1, reinterpret_cast<void *>(img->data_));
      if ((spinnaker_driver::SupportedPixelFormat::PIXEL_FORMAT_GB8 == img->pixel_format_) ||
        (spinnaker_driver::SupportedPixelFormat::PIXEL_FORMAT_RG8 == img->pixel_format_))
      {
        cv::Mat frameBGR8 = cv::Mat(
          static_cast<int>(img->height_), static_cast<int>(img->width_), CV_8UC3);
        if (spinnaker_driver::SupportedPixelFormat::PIXEL_FORMAT_GB8 == img->pixel_format_) {
          cv::cvtColor(frame, frameBGR8, CV_BayerBG2BGR);
        } else {
          cv::cvtColor(frame, frameBGR8, CV_BayerGR2BGR);
        }
        frame = frameBGR8.clone();
      }
      cv::Mat display_frame = cv::Mat();
      cv::resize(frame, display_frame, cv::Size(), 0.5, 0.5);
      // cv::imshow("Image", display_frame);
      // cv::waitKey(100);    // otherwise the image will not display...
      std::string fname = "gigev_acqustion_";
      fname += std::to_string(count) + ".png";
      cv::imwrite(fname, display_frame);
    } else {
      std::cout << "image not successfully acquired" << std::endl << std::flush;
    }

    timestamp_ = img->timestamp_;
    count++;
  }
  uint64_t timestamp_;
};

void spin(spinnaker_driver::SpinnakerDriverGigE * gigev)
{
  using namespace std::chrono_literals;
  static const std::chrono::seconds kRunningThreadTimeout(10);

  std::this_thread::sleep_for(kRunningThreadTimeout);

  std::cout << "exiting from thread" << std::endl << std::flush;
  gigev->stop();
}

int main(int argc, char * argv[])
{
  bool list_cameras = false;
  uint32_t camera_index = 0;
  bool connect_camera = false;
  bool update_frame_rate = false;
  float frame_rate = 1.0f;

  image_capture capture;
  spinnaker_driver::AcquiredImageCallback acquisition_callback =
    std::bind(&image_capture::acquired, &capture, std::placeholders::_1);

  // arguments parsing loop
  std::vector<std::string> args(argv + 1, argv + argc);
  for (auto i = args.begin(); i != args.end(); i++) {
    if ( ("-h" == *i) || ("--help" == *i) ) {
      std::cout << argv[0] << "-l [list cameras] || -c <camera> || -f <frame rate>" << std::endl;
      std::cout << "-f should be used along with -c" << std::endl;
      return 0;
    }
    // List camera, no further argument is expected
    if ("-l" == *i) {
      list_cameras = true;
      break;
    }
    // Connect to specified camera. One more argument is expected
    if ("-c" == *i) {
      connect_camera = true;
      // Get camera index when specified
      try {
        uint32_t which = (uint32_t)std::stoul(*++i);
        camera_index = which;
      } catch (const std::exception & e) {
        std::cerr << e.what() << std::endl << "use 0 for camera index" << std::endl;
      }
    }
    // Frame rate set. Expect another argument for frame rate (1 as 1 FPS, 100 as 100 FPS)
    if ("-f" == *i) {
      update_frame_rate = true;
      try {
        float what = std::stof(*++i);
        frame_rate = what;
        std::cout << "Frame rate specified: " << what << std::endl;
      } catch (const std::exception & e) {
        std::cerr << e.what() << std::endl << "use 1 (FPS) for frame rate" << std::endl;
      }
    }
  }

  // Instantiate GigEV driver
  spinnaker_driver::SpinnakerDriverGigE gigev_driver;
  if (list_cameras) {
    bool success = gigev_driver.list(spinnaker_driver::SupportedCameraType::CAMERA_TYPE_GIGE);
    if (!success) {
      std::cout << "Failed to list GigEV cameras" << std::endl;
      return -1;
    }
  }

  if (connect_camera) {
    std::cout << "connecting to camera " << camera_index << std::endl;
    if (gigev_driver.connect(camera_index)) {
      // start a process running for 10s
      std::shared_ptr<std::thread> spin_thread = std::make_shared<std::thread>(
        &spin, &gigev_driver);
      bool thread_status = false;
      if (update_frame_rate) {
        thread_status = gigev_driver.start(acquisition_callback, frame_rate);
      } else {
        thread_status = gigev_driver.start(acquisition_callback);
      }
      if (thread_status) {
        std::cout << "Acquistion is successfully terminated" << std::endl;
      } else {
        std::cout << "Failed to start camera: " << camera_index << std::endl;
      }
      if (spin_thread->joinable()) {
        spin_thread->join();
        std::cout << "User spin thread terminated0" << std::endl;
      }
      std::cout << "User spin thread terminated1" << std::endl << std::flush;
    } else {
      std::cout << "Failed to connect camera: " << camera_index << std::endl;
    }
  }

  if (list_cameras || connect_camera) {
    try {
      gigev_driver.release();
    } catch (const std::exception & e) {
      std::cerr << "User forced gigev_driver.release exception: " << e.what() << '\n';
    }
  }

  std::cout << std::flush;
  return 0;
}
