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

#ifndef SPINNAKER_DRIVER__APPS__MACHINE_VISION_STREAMING_HPP_
#define SPINNAKER_DRIVER__APPS__MACHINE_VISION_STREAMING_HPP_

// c standard headers
#include <stdint.h>
#include <stdbool.h>

// gstreamer headers
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// std headers
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <chrono>
#include <mutex>
#include <condition_variable>

// opencv headers
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "image_capture.hpp"

#pragma mark - VideoStream
class VideoStream : public image_capture
{
public:
  struct VideoStreamProperty
  {
    uint32_t width;
    uint32_t height;
    /**
     * @brief pixel format used to build the pipleine
     */
    std::string pixel_format;
    /**
     * @brief client (streaming to) IP address
     */
    std::string client_ip;
    /**
     * @brief client port number
     */
    uint32_t client_port;

    /**
     * @brief frame rate
     */
    float frame_rate;

    /**
     * @brief Timestamp
     */
    GstClockTime timestamp;

    /**
     * @brief main loop used to stop the pipeline
     */
    GMainLoop * loop;

    /**
     * @brief appsrc mainly used to push streaming buffer
     */
    GstElement * appsrc;
    // TODO(tcao): Use std::function such that this accessible
    void ( * appsrc_logic)(const cv::Mat & input, cv::Mat & output);
  };

  /**
   * @brief Construct a new Video Stream object
   */
  VideoStream();

  /**
   * @brief Destroy the Video Stream object
   */
  virtual ~VideoStream();

  /**
   * @brief To star/stop receving thread
   *
   * @param start - true to start, false to stop
   * @remark This is single threaded image receiving processor
   */
  void receive(bool start = true);

  /**
   * @brief To start/stop sending thread
   *
   * @param start - true to start, false to stop
   * @remark This is multithreaded image visualization and analytic processor
   */
  void send(bool start = true);

  /**
   * @brief Stop streaming
   */
  void stop();

  /**
   * @brief Register user provided streaming pipelines
   *
   * @param properties - Pipeline property details
   * @param property_count - Count of pipeline properties
   */
  void register_streams(VideoStreamProperty * properties, uint32_t property_count);

protected:
  /**
   * @brief Receiving images - pipeline starting point
   */
  void receiving();

  /**
   * @brief Configure and run UDP sink sending pipeline
   *
   * @param loop - Main loop handle
   * @param property - pipeline resolution, pixel format, etc.
   *
   * @remark This code is designed to run multiple times in one session to support stream multicast
   */
  void configure_run_sending_pipeline(VideoStreamProperty * property);

  /**
   * @brief Push image buffer on-demand down to streaming pipeline
   *
   * @param image Assuming processed image to be pushed
   * @param property Some important parameters for this pipeline
   * @remark It runs in multi-thread environment
   */
  void push(cv::Mat & image, VideoStreamProperty * property);

  /**
   * @brief Register a pipeline property
   *
   * @param property
   */
  void register_stream(VideoStreamProperty * property);

  /**
   * @brief Override base image_capture
   *
   */
  void image_release() override;

  /**
   * @brief Receiving thread looping exit flag
   */
  bool image_acquisition_exit_;

  /**
   * @brief Synchronization between receiving thread and sending thread(s)
   * @remark Used along with std::condition_variable frame_ready_condition_
   */
  std::mutex frame_ready_mutex_;

  /**
   * @brief Synchronization between receiving thread and sending thread(s)
   * @remark Used along with std::mutex frame_ready_mutex_
   */
  std::condition_variable frame_ready_condition_;
  /**
   * @brief Synchronization between sending threads to access shared resources
   */
  std::mutex critical_section_;

  /**
   * @brief Containers for provided user pipeline details
   */
  std::vector<VideoStreamProperty *> properties_;
};

#endif  // SPINNAKER_DRIVER__APPS__MACHINE_VISION_STREAMING_HPP_
