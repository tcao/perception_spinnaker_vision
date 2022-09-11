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

#include "machine_vision_streaming.hpp"

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

// Local CV algorithms
#include "./edgedetector.h"

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

#pragma mark - VideoStream Implementation
VideoStream::VideoStream()
: image_acquisition_exit_(false)
{}

VideoStream::~VideoStream()
{
  while (0 < properties_.size()) {
    properties_.pop_back();
  }
}

void VideoStream::receive(bool start)
{
  g_message("VideoStream::receive(%s)", start ? "true" : "false");
  if (!start) {
    image_acquisition_exit_ = true;
    return;
  }

  // Start receiving thread
  std::thread image_acquisition_thread_ = std::thread(&VideoStream::receiving, this);
  image_acquisition_thread_.detach();
}

/** protected */
void VideoStream::receiving()
{
  g_message("VideoStream::receiving");
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
      for (VideoStreamProperty * property : this->properties_) {
        property->appsrc_logic(image_, converted_buffer[index]);
        push(converted_buffer[index], property);
        index++;
      }
    } catch (std::exception & e) {
      std::cerr << "Receiving thread error: " << e.what() << std::endl;
    }
    count++;
  }
}

void VideoStream::send(bool start)
{
  g_message("VideoStream::send(%s)", start ? "true" : "false");

  for (VideoStreamProperty * property : this->properties_) {
    if (!start) {
      if (property->loop) {
        g_main_loop_quit(property->loop);
        property->loop = nullptr;
      }
    } else {
      // Create a new context so resources can attach (instead of default main thread)
      property->loop = g_main_loop_new(g_main_context_new(), FALSE);
      std::thread(&VideoStream::configure_run_sending_pipeline, this, property).detach();
    }
  }
}

/** protected
 *  @remark This is multi-threaded routine
 */
void VideoStream::configure_run_sending_pipeline(VideoStreamProperty * property)
{
  g_message(
    "VideoStream::configure_run_sending_pipeline: 0x%lx",
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
    //  "video/x-raw,format=BGRA,height=" STRING(HEIGHT) ",width=" STRING(WIDTH) ",framerate=30/1")
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
    "VideoStream::configure_run_sending_pipeline running: 0x%lx\n",
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
    "VideoStream::configure_run_sending_pipeline exits: 0x%lx",
    reinterpret_cast<uintptr_t>(pipeline));
  critical_section_.unlock();

  gst_object_unref(pipeline);
}

/** protected
 *  @remark This is multi-threaded routine
 */
void VideoStream::push(cv::Mat & image, VideoStreamProperty * property)
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

  // gst_app_src_push_buffer takes ownership of the buffer, so no need for gst_buffer_unref (buffer)
  if (GST_FLOW_OK != gst_app_src_push_buffer(GST_APP_SRC(property->appsrc), buffer)) {
    g_message("Unable to push buffer");
  }
}

void VideoStream::stop()
{
  g_message("VideoStream::stop");
  receive(false);
  send(false);
}

void VideoStream::register_streams(VideoStreamProperty * properties, uint32_t property_count)
{
  for (uint32_t i = 0; i < property_count; i++) {
    VideoStreamProperty * property = &properties[i];
    register_stream(property);
  }
}

/** protected */
void VideoStream::register_stream(VideoStreamProperty * property)
{
  properties_.push_back(property);
}

/**
 * This is called from base image_capture::acquired
 * protected and override
 */
void VideoStream::image_release()
{
  // Notify receiving thread data is updated and ready
  std::lock_guard<std::mutex> lk(this->frame_ready_mutex_);
  frame_ready_condition_.notify_one();
}


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
 * @param video_stream
 * @param parameters
 */
void setup_pipelines(std::shared_ptr<VideoStream> video_stream, cli_parameters * parameters)
{
  const uint32_t WHEN_TO_SAVE_DEBUG_IMAGE(62);  // Save input images in this successful capture
  const uint32_t EACH_FILTER_RUNS(30);          // Each filter runs this many times

  uint32_t udp_port = parameters->first_port;
  static VideoStream::VideoStreamProperty streams[] = {
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
      .appsrc_logic = [](const cv::Mat & input, cv::Mat & output) {
          // Not doing anything, just pass along w/o affecting passing input
          static uint32_t count = 0;
          if (WHEN_TO_SAVE_DEBUG_IMAGE == count) {
            std::cout << "BGR - saving image in the format of: " << input.type() << std::endl;
            cv::imwrite("mv_bgr.png", input);
          }
          count++;
          output = input.clone();
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
      .appsrc_logic = [](const cv::Mat & input, cv::Mat & output) {
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
            // Clone it since it is already in the proper format
            if (WHEN_TO_SAVE_DEBUG_IMAGE == count) {
              std::cout << "GRAY8_0 - saving image in the format of: " << input.type() << std::endl;
              cv::imwrite("mv_gry.png", input);
            }
            count++;
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
        },
    },
  };

  video_stream->register_streams(streams, sizeof(streams) / sizeof(streams[0]));
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
    std::cout << argv[0] << " [-c <camera>] [-h <host IP>] [--help]" << std::endl;
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
    .camera = CLI_DEFAULT_CAMERA,
    // Don't care about frame rate, it is filled by the system
    .frame_rate = 1.0f,
  };

  if (!argument_parse(argc, argv, &parameters)) {
    return 0;
  }

  std::shared_ptr<VideoStream> video_stream = std::make_shared<VideoStream>();
  // The receiving pipeline doesn't need camera specific info
  video_stream->receive();

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
      std::bind(&VideoStream::acquired, video_stream, std::placeholders::_1);

    // Fill corresponding setup parameter with camera's current parameters
    parameters.dimension = camera_parameters.image_dimension;
    parameters.frame_rate = camera_parameters.frame_rate;

    setup_pipelines(video_stream, &parameters);

    // Start streaming sending pipelines
    video_stream->send();

    // This demonstration - exiting after specified seconds
    std::thread spin_thread([&gigev_driver, video_stream]() {
        // Run the streaming for this amount of time in seconds
        std::this_thread::sleep_for(std::chrono::seconds(120));
        gigev_driver.stop();
        video_stream->stop();
      });

    // Start image acquisition, no return till forced to
    bool thread_status = gigev_driver.start(acquisition_callback, parameters.frame_rate);
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
