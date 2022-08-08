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
/// FLIR camera driver based on Spinnaker SDK
/// @remark The current implementation covers only GigEV camera
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///
#ifndef SPINNAKER_DRIVER__SPINNAKER_DRIVER_HPP_
#define SPINNAKER_DRIVER__SPINNAKER_DRIVER_HPP_

#include <stdint.h>
#include <stdbool.h>
#include <memory>
#include <functional>
#include <thread>

namespace spinnaker_driver
{
// @remark OpenCV default Pixel format is BGR8
enum SupportedPixelFormat
{
  PIXEL_FORMAT_MONO8,   // PixelFormat_Mono8 (GS3-PGE-60S6M) ->
  PIXEL_FORMAT_GB8,     // PixelFormat_BayerGB8 (BFLY-PGE-50S5C) -> cvtColor CV_BayerGR2BGR
  PIXEL_FORMAT_RG8,     // PixelFormat_BayerRG8 -> cvtColor CV_BayerBG2BGR
                        // According AcquisionOpenCV.cpp:
                        // 1) pResultImage->Convert(PixelFormat_RGB8, HQ_LINEAR)
                        // 2) pass PixelFormat_BayerRG8 to OpenCV, then cvtColor COLOR_BayerRG2RGB
  PIXEL_FORMAT_BGR8,    // PixelFormat_BGR8
  PIXEL_FORMAT_RGB8,    // PixelFormat_RGB8
  PIXEL_FORMAT_INVALID,
};

enum SupportedCameraType
{
  CAMERA_TYPE_ALL,      // Any type
  CAMERA_TYPE_GIGE,     // Supported
  CAMERA_TYPE_U3V,      // Not yet supported
};

struct AcquiredImage
{
  AcquiredImage()
  : timestamp_(0),
    width_(0),
    height_(0),
    x_padding_(0),
    y_padding_(0),
    stride_(0),
    bits_per_pixel_(0),
    channels_(0),
    // pixel_format_(SupportedPixelFormat::PIXEL_FORMAT_INVALID),
    pixel_format_(0),
    data_(nullptr)
  {}
  uint64_t timestamp_;
  size_t width_;
  size_t height_;
  size_t x_padding_;
  size_t y_padding_;
  size_t stride_;
  size_t bits_per_pixel_;
  size_t channels_;
  // SupportedPixelFormat pixel_format_;
  uint16_t pixel_format_;
  /**
   * @brief image data buffer
   * @remark if null, it indicates data is not complete
   */
  const void * data_;
};
typedef std::shared_ptr<AcquiredImage> AcquiredImagePtr;
typedef std::shared_ptr<const AcquiredImage> AcquiredImageConstPtr;
typedef std::function<void (const AcquiredImage * img)> AcquiredImageCallback;

class SpinnakerDriver
{
public:
  /**
   * @brief Construct a new spinnaker driver object
   *
   */
  SpinnakerDriver();
  /**
   * @brief Destroy the spinnaker driver object
   *
   */
  virtual ~SpinnakerDriver();

  /**
   * @brief Connect to the specified camera
   *
   * @param camera_type type of camera - current it supports only Spinnaker::InterfaceTypeEnum::InterfaceType_GigEVision
   *  Note default argument value is not part of signature
   * @param camera_index which camera to connect, default to the first camera
   * @return true when specified camera is successfully connected
   * @return false failed to connect specified camera, or wrong index is provided
   *
   * @note The same GigE camera will be listed multiple times if the host
   * has multiple ethernet adapters. It is therefore important to choose the proper one.
   *
   * @note camera_type uses general type instead of Spinnaker::InterfaceTypeEnum
   * @note camera_index is the camera index to found camera list
   */
  virtual bool connect(
    uint32_t camera_index = 0,
    SupportedCameraType camera_type = CAMERA_TYPE_ALL) = 0;

  /**
   * @brief Start camera
   *
   * @param acquisition_callback callback when image event occurs
   * @param frame_rate desired rate (default to 1.0)
   * @remark if desired frame_rate exceeds allowed rate, it will be adjusted to most reasonable one
   * @return true when start operation is successful
   * @return false when start operation fails
   */
  virtual bool start(AcquiredImageCallback acquisition_callback, float frame_rate = 1.0f) = 0;

  /**
   * @brief Stop camera image acquisition
   *
   * @return true when stop operation is successful
   * @return false when stop operation fails
   */
  virtual bool stop() = 0;

  /**
   * @brief List camera(s) for the specified camera type
   *
   * @param camera_type Camera type
   * @return true when successful
   * @return false when fail
   *
   * @note The console output information contains camera index
   *  In GigE camera, the same camera may show up multitple times,
   *  so it is important to specified which camera (from which interface)
   */
  virtual bool list(SupportedCameraType camera_type = CAMERA_TYPE_ALL) = 0;

  /**
   * @brief Release driver instance
   *
   */
  virtual void release() = 0;

protected:
  /**
   * @brief Internal control loop routine to get image acquisition events alive
   * @remark Subclass should provide a threading model
   */
  virtual void run() = 0;
};

class SpinnakerDriverGigE : public SpinnakerDriver
{
public:
  SpinnakerDriverGigE();
  virtual ~SpinnakerDriverGigE();
  bool connect(
    uint32_t camera_index = 0, SupportedCameraType camera_type = CAMERA_TYPE_GIGE) override;
  bool start(AcquiredImageCallback acquisition_callback, float frame_rate = 1.0f) override;
  bool stop() override;
  bool list(SupportedCameraType camera_type = CAMERA_TYPE_GIGE) override;
  void release() override;

protected:
  void run() override;
  uint32_t cameraIndex_;
  AcquiredImageCallback acquisitionCallback_;
  /**
   * @brief The thread runs in 10Hz or 100ms frequency
   */
  std::shared_ptr<std::thread> runningThread_;
  volatile bool running_;
};

}  // namespace spinnaker_driver
#endif  // SPINNAKER_DRIVER__SPINNAKER_DRIVER_HPP_
