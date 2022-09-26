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
/// GigEV camera image capture callback wrapper
/// @remark Callback wrapper for image acquisition/machine vision application
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

#ifndef SPINNAKER_PERCEPTION_APPS__IMAGE_CAPTURE_HPP_
#define SPINNAKER_PERCEPTION_APPS__IMAGE_CAPTURE_HPP_
// STD headers
#include <string>

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

// Spinnaker driver header
#include "spinnaker_driver/spinnaker_driver.hpp"

struct image_capture
{
public:
  image_capture(
    void(*handler)(const cv::Mat & input) = nullptr,
    uint64_t timestamp = 0, bool debug = false)
  : image_handler(handler),
    timestamp_(timestamp),
    debug_(debug)
  {}

  void acquired(const spinnaker_driver::AcquiredImage * img)
  {
    static uint32_t count = 0;
    std::chrono::system_clock::time_point current_timestamp{
      std::chrono::system_clock::duration{img->timestamp_}};
    if (debug_) {
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
    }
    if (nullptr != img->data_) {
      if (debug_) {
        std::cout << "pixel format: " << img->pixel_format_ << std::endl;
        std::cout << "data: " << std::hex << img->data_ << std::endl;
        std::cout << std::dec << std::endl << std::flush;
      }

      // When Bayer pattern is used (as CCD/CMOS usually does),
      // raw data is organized in single channel, conversion from CV_8UC1 to CV_8UC3 is needed
      cv::Mat frame = cv::Mat(
        static_cast<int>(img->height_), static_cast<int>(img->width_),
        (3 == img->channels_) ? CV_8UC3 : CV_8UC1,
        // @remark No data is allocated, it just points to provided data
        const_cast<void *>(reinterpret_cast<const void *>(img->data_)),
        // TODO(tcao): Mono8 format has stride, but use or not doesn't seem matter
        img->stride_);
      if (spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_MONO8 == img->pixel_format_) {
        // Clone the image even it is "identical" to the original one which holds SDK buffer
        image_ = frame.clone();
      } else {
        bool converted = false;
        if ((spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_GB8 == img->pixel_format_) ||
          (spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_RG8 == img->pixel_format_))
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
        if (spinnaker_driver::SupportedPixelFormat_e::PIXEL_FORMAT_BGR8 == img->pixel_format_) {
          // frame is already in proper format, as in the above in case of PIXEL_FORMAT_MONO8
          converted = true;
          image_ = frame.clone();
        }

        if (!converted) {
          std::cerr << "Unable to convert, please add the format (" << img->pixel_format_ <<
            ") to the support list" << std::endl;
        }
      }

      // Let image processor to apply application logic
      if (image_handler) {
        // the frame may contain unsupported format, let it go even it can't be display properly
        image_handler(image_);
      }

      // Notify receiving thread to consume the acquired image,
      // including potential scaling
      image_release();
    } else {
      // Either the captured image is incomplete or pixel format is not supported
      std::cerr << "image not successfully acquired" << std::endl << std::flush;
    }

    timestamp_ = img->timestamp_;
    count++;
  }

  /**
   * @brief A function pointer means to hookup application to process acquired image
   */
  void ( * image_handler)(const cv::Mat & input);

  /**
   * @brief To release acquired image
   * @remark It is intended for the subclass to use
   */
  virtual void image_release() {}

  /**
   * @brief Captured image instance
   */
  cv::Mat image_;

  /**
   * @brief Internal time stamp
   */
  uint64_t timestamp_;

  bool debug_;
};

#endif  // SPINNAKER_PERCEPTION_APPS__IMAGE_CAPTURE_HPP_
