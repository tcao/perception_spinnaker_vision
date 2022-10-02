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

#ifndef SPINNAKER_PERCEPTION_APPS__VISION_STREAMING_HPP_
#define SPINNAKER_PERCEPTION_APPS__VISION_STREAMING_HPP_

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

namespace
{
#pragma mark - Local Macros
// Stringizing the token
#define CREATE_GST_ELEMENT0(a) #a
// Concatenating 2 tokens
#define CREATE_GST_ELEMENT1(a, b)  CREATE_GST_ELEMENT0(a ## b)
#define CREATE_GST_ELEMENT(element) \
  GstElement * element = gst_element_factory_make( \
    #element, \
    CREATE_GST_ELEMENT1(element, _name) \
    );
}  // anonymous namespace

namespace spinnaker_perception_apps
{
class VisionStream : public image_capture
{
public:
  struct VisionStreamProperty
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
    /**
     * @brief Callback for acquired images
     *
     * @param input - Input image. Don't hold it per Spinnaker
     * @param output - Output image
     *
     * @return true - Don't ignore output to do further processing
     * @return false - Ignore output
     */
    bool ( * appsrc_logic)(const cv::Mat & input, cv::Mat & output);
  };

  /**
   * @brief Construct a new Video Stream object
   */
  VisionStream()
  : image_acquisition_exit_(false)
  {
    // image_capture is instantiated with default parameters
  }

  /**
   * @brief Destroy the Video Stream object
   */
  virtual ~VisionStream()
  {
    while (0 < properties_.size()) {
      properties_.pop_back();
    }
  }

  /**
   * @brief To star/stop receving thread
   *
   * @param start - true to start, false to stop
   * @remark This is single threaded image receiving processor
   */
  void receive(bool start = true)
  {
    g_message("VisionStream::receive(%s)", start ? "true" : "false");
    if (!start) {
      image_acquisition_exit_ = true;

      // Abort lock to force exiting from detached thread receiving
      image_release();
      return;
    }

    // Start receiving thread
    std::thread image_acquisition_thread_ = std::thread(&VisionStream::receiving, this);
    image_acquisition_thread_.detach();
  }


  /**
   * @brief To start/stop sending thread
   *
   * @param start - true to start, false to stop
   * @remark This is multithreaded image visualization and analytic processor
   */
  void send(bool start = true)
  {
    g_message("VisionStream::send(%s)", start ? "true" : "false");

    for (VisionStreamProperty * property : this->properties_) {
      if (!start) {
        if (property->loop) {
          g_main_loop_quit(property->loop);
          property->loop = nullptr;
        }
      } else {
        // Create a new context so resources can attach (instead of default main thread)
        property->loop = g_main_loop_new(g_main_context_new(), FALSE);
        std::thread(&VisionStream::configure_run_sending_pipeline, this, property).detach();
      }
    }
  }

  /**
   * @brief Stop streaming
   */
  void stop()
  {
    g_message("VisionStream::stop");
    receive(false);
    send(false);
  }

  /**
   * @brief Register user provided streaming pipelines
   *
   * @param properties - Pipeline property details
   * @param property_count - Count of pipeline properties
   */
  void register_streams(VisionStreamProperty * properties, uint32_t property_count)
  {
    for (uint32_t i = 0; i < property_count; i++) {
      VisionStreamProperty * property = &properties[i];
      register_stream(property);
    }
  }

protected:
  /**
   * @brief Receiving images - pipeline starting point
   */
  void receiving()
  {
    g_message("VisionStream::receiving");
    image_acquisition_exit_ = false;
    static uint32_t count = 0;
    // TODO(tcao): Fix the hard coded constant, and use this->properties_.size() instead of 2
    // Currently, when this->properties_.size() is used,
    // 1 - It gets a compiling warning:
    // "warning: variable length arrays are a C99 feature"
    // 2 - It gets a runtime exception:
    // "pointer being freed was not allocated"
    cv::Mat converted_buffer[2];

    while (!image_acquisition_exit_) {
      // unique_lock instead of lock_guard is used because of conditional_variable's
      // nature which performs lock and unlock during its wait
      std::unique_lock<std::mutex> guard(this->frame_ready_mutex_);
      // The loop's running rate is decided by whoever release frame_ready_condition_
      frame_ready_condition_.wait(guard);

      // image_acquisition_exit_ could be changed after the above wait,
      // which could be forced to abort, so let's check image_acquisition_exit_ again
      if (image_acquisition_exit_) {
        break;
      }

      // Useful debugging info, in particular timestamp showing frame rate
      g_message("receiving: %u", count);

      // Retrieve image and scale it if necessary
      // Assumption: the same image width/height applies to app pipeline
      // otherwise the scaling logic should be within the for loop for each pipeline
      if (0 < this->properties_.size()) {
        uint32_t width = properties_[0]->width;
        uint32_t height = properties_[0]->height;
        if ( (width != static_cast<uint32_t>(image_.cols)) ||
          (height != static_cast<uint32_t>(image_.rows)))
        {
          // Usually it is a shrink operation, the best option is INTER_AREA for decimation
          cv::resize(image_, image_, cv::Size(width, height), 0, 0, cv::INTER_AREA);
        }
      }
      try {
        // push retrieved data to each stream
        uint32_t index = 0;
        for (VisionStreamProperty * property : this->properties_) {
          if (property->appsrc_logic(image_, converted_buffer[index])) {
            // @remark - This (skipping the frame when appsrc_logic returns false)
            // will screw up streaming timing
            push(converted_buffer[index], property);
          }
          index++;
        }
      } catch (std::exception & e) {
        std::cerr << "Receiving thread error: " << e.what() << std::endl;
      }
      count++;
    }
    g_message("VisionStream::receiving exits");
  }

  /**
   * @brief Configure and run UDP sink sending pipeline
   *
   * @param loop - Main loop handle
   * @param property - pipeline resolution, pixel format, etc.
   *
   * @remark This code is designed to run multiple times in one session to support stream multicast
   */
  void configure_run_sending_pipeline(VisionStreamProperty * property)
  {
    g_message(
      "VisionStream::configure_run_sending_pipeline: 0x%lx",
      reinterpret_cast<uintptr_t>(property->loop));
    gst_init(nullptr, nullptr);
    GstElement * pipeline = gst_pipeline_new("pipeline");
#if VIDEOTESTSRC
    CREATE_GST_ELEMENT(videotestsrc);
    if (!videotestsrc) {
      g_error("gst_element_factory_make failed with videotestsrc");
    }
#else
    CREATE_GST_ELEMENT(appsrc);
    if (!appsrc) {
      g_error("gst_element_factory_make failed with appsrc");
    }
    g_object_set(
      G_OBJECT(appsrc), "caps",
      // gst_caps_new_simple is equivalent to:
      // gst_caps_from_string(
      // "video/x-raw,format=BGRA,height=" STRING(HEIGHT) ",width=" STRING(WIDTH) ",framerate=30/1")
      // where (https://lists.freedesktop.org/archives/gstreamer-devel/2017-September/065496.html)
      gst_caps_new_simple(
        "video/x-raw",
        // format doesn't seem is absolutely necessary, default 0 - GST_FORMAT_UNDEFINED seems OK
        "format", G_TYPE_STRING, property->pixel_format.c_str(),
        "width", G_TYPE_INT, static_cast<int>(property->width),
        "height", G_TYPE_INT, static_cast<int>(property->height),
        // 0/1 is variable frame rate
        "framerate", GST_TYPE_FRACTION, 0, 1,
        NULL),
      NULL);
    g_object_set(
      G_OBJECT(appsrc),
      /*
        stream-type : the type of the stream
                    flags: readable, writable
                    Enum "GstAppStreamType" Default: 0, "stream"
                      (0): stream           - GST_APP_STREAM_TYPE_STREAM
                      (1): seekable         - GST_APP_STREAM_TYPE_SEEKABLE
                      (2): random-access    - GST_APP_STREAM_TYPE_RANDOM_ACCESS
      */
      "stream-type", 0,   // stream
      "is-live", true,
      "format", GST_FORMAT_TIME,
      NULL);
#endif
    // videoconvert is necessary with videotestsrc
    CREATE_GST_ELEMENT(videoconvert);
    if (!videoconvert) {
      g_error("gst_element_factory_make failed with videoconvert");
    }

    CREATE_GST_ELEMENT(x264enc);
    if (!x264enc) {
      g_error("gst_element_factory_make failed with x264enc");
    }
    g_object_set(
      G_OBJECT(x264enc),
      /*
        tune : Preset name for non-psychovisual tuning options
          flags: readable, writable
          Flags "GstX264EncTune" Default: 0x00000000, "(none)"
          (0x00000001): stillimage       - Still image
          (0x00000002): fastdecode       - Fast decode
          (0x00000004): zerolatency      - Zero latency
      */
      /*
        By default x264enc will aim for best quality rather than low latency,
        which means it may consume a couple of seconds of video before
        outputting anything. In this case it will consume data faster than
        real-time at the beginning. Eventually it will be throttled by the sink
        syncing to the clock
        tune=zerolatency to make it not do that.
      */
      "tune", 4,        // zerolatency
      "bitrate", 500,
      /*
        speed-preset  : Preset name for speed/quality tradeoff options (can affect decode compatibility - impose restrictions separately for your target decoder)
                      flags: readable, writable
                      Enum "GstX264EncPreset" Default: 6, "medium"
                        (0): None             - No preset
                        (1): ultrafast        - ultrafast
                        (2): superfast        - superfast
                        (3): veryfast         - veryfast
                        (4): faster           - faster
                        (5): fast             - fast
                        (6): medium           - medium
                        (7): slow             - slow
                        (8): slower           - slower
                        (9): veryslow         - veryslow
                        (10): placebo          - placebo
      */
      "speed-preset", 2,  // superfast
      NULL);

    CREATE_GST_ELEMENT(rtph264pay);
    if (!rtph264pay) {
      g_error("gst_element_factory_make failed with rtph264pay");
    }
    // This is important to let VLC knows the config
    // https://stackoverflow.com/questions/57824619/how-to-stream-video-over-udp-from-gstreamer-1-0-to-vlc
    g_object_set(
      G_OBJECT(rtph264pay),
      "config-interval", 60,
      NULL);

    CREATE_GST_ELEMENT(udpsink);
    if (!udpsink) {
      g_error("gst_element_factory_make failed with udpsink");
    }
    g_object_set(
      G_OBJECT(udpsink),
      "host", property->client_ip.c_str(),
      "port", property->client_port,
      NULL);
    // async default to true, sync (on clock) default to true

#if VIDEOTESTSRC
    gst_bin_add_many(
      GST_BIN(pipeline), videotestsrc, videoconvert, x264enc, rtph264pay, udpsink, NULL);
    gst_element_link_many(videotestsrc, videoconvert, x264enc, rtph264pay, udpsink, NULL);
#else
    gst_bin_add_many(GST_BIN(pipeline), appsrc, videoconvert, x264enc, rtph264pay, udpsink, NULL);
    gst_element_link_many(appsrc, videoconvert, x264enc, rtph264pay, udpsink, NULL);
#endif

    // Change pipeline state to PLAYING
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    // @remark Use the following blocking code seems stall the system
    // GstStateChangeReturn pipeline_status = gst_element_get_state (pipeline, NULL, NULL, -1);
    // if (GST_STATE_CHANGE_SUCCESS != pipeline_status)

    // Enter critical section
    critical_section_.lock();
#if VIDEOTESTSRC
    // @remark This is used for pipeline testing
    std::cout <<
      "videotestsrc ! videoconvert ! x264enc tune=4 bitrate=500 ! rtph264pay ! udpsink host=" <<
      property->client_ip.c_str() << " port=" << property->client_port << std::endl << std::endl;
    std::cout << "Use the following pipeline to receive/display:" << std::endl;
    std::cout << "gst-launch-1.0 -v udpsrc port=" << property->client_port <<
      " caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\"" <<
      " ! rtph264depay ! decodebin ! videoconvert ! autovideosink sync=false" << std::endl;
#else
    std::cout <<
      "appsrc stream-type=0,is-live=true,format=3 ! video/x-raw,foramt=" <<
      property->pixel_format.c_str() <<
      ",width=" << property->width << ",height=" << property->height << ",framerate=0/1" <<
      " ! videoconvert ! x264enc tune=4 bitrate=500 speed-preset=2 ! rtph264pay ! udpsink host=" <<
      property->client_ip.c_str() << " port=" << property->client_port << std::endl << std::endl;
    std::cout << "Use the following pipelines to receive:" << std::endl;
    std::cout << "gst-launch-1.0 -v udpsrc port=" << property->client_port <<
      " caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\"" <<
      " ! rtph264depay ! decodebin ! videoconvert ! autovideosink sync=false" <<
      std::endl << std::endl;
#endif
    std::cout << std::flush;

    g_message(
      "VisionStream::configure_run_sending_pipeline running: 0x%lx\n",
      reinterpret_cast<uintptr_t>(pipeline));

    // Exit critical section
    critical_section_.unlock();

    // The local stack copy is used for further operation
    property->appsrc = appsrc;

    property->timestamp = 0;

    // No return till a force exit by calling g_main_loop_quit
    g_main_loop_run(property->loop);

    // Thread resumes here after exiting from g_main_loop_run
    gst_element_set_state(pipeline, GST_STATE_NULL);

    critical_section_.lock();
    g_message(
      "VisionStream::configure_run_sending_pipeline exits: 0x%lx",
      reinterpret_cast<uintptr_t>(pipeline));
    critical_section_.unlock();

    gst_object_unref(pipeline);
  }

  /**
   * @brief Push image buffer on-demand down to streaming pipeline
   *
   * @param image Assuming processed image to be pushed
   * @param property Some important parameters for this pipeline
   * @remark It runs in multi-thread environment
   */
  void push(cv::Mat & image, VisionStreamProperty * property)
  {
    guint size = image.total() * image.elemSize();
    GstBuffer * buffer = gst_buffer_new_wrapped_full(
      GST_MEMORY_FLAG_READONLY,
      static_cast<gpointer>(image.data), size, 0, size,
      nullptr, nullptr);

    GST_BUFFER_PTS(buffer) = property->timestamp;

    // @remark This affects framerate
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(
      1, GST_SECOND, round(property->frame_rate));

    property->timestamp += GST_BUFFER_DURATION(buffer);

    // gst_app_src_push_buffer takes ownership of the buffer,
    // so no need for gst_buffer_unref (buffer)
    if (GST_FLOW_OK != gst_app_src_push_buffer(GST_APP_SRC(property->appsrc), buffer)) {
      g_message("Unable to push buffer");
    }
  }


  /**
   * @brief Register a pipeline property
   *
   * @param property
   */
  void register_stream(VisionStreamProperty * property)
  {
    properties_.push_back(property);
  }

  /**
   * @brief Override base image_capture
   *
   * @remark This is called from base image_capture::acquired
   */
  void image_release() override
  {
    // Notify receiving thread data is updated and ready
    std::lock_guard<std::mutex> lk(this->frame_ready_mutex_);
    frame_ready_condition_.notify_one();
  }

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
  std::vector<VisionStreamProperty *> properties_;
};
}  // namespace spinnaker_perception_apps
#endif  // SPINNAKER_PERCEPTION_APPS__VISION_STREAMING_HPP_
