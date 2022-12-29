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
#include <mutex>
#include <condition_variable>
#include <filesystem>

#include "spinnaker_driver/spinnaker_driver.hpp"
#include "spinnaker_ros2/spinnaker_ros2.hpp"

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

#define ENABLE_SENSOR_TOPIC 0

namespace
{
static const char defaultNodeName[] = "spinnaker_ros2";
static const char defaultImageTopicName[] = "image";
static const char defaultCalibrationXml[] = "gigev_calib.xml";
static const uint32_t defaultCamera(0);
static const float defaultGridDimensionScale(1.0f);
static const uint32_t defaultGridWithCount(9);
static const uint32_t defaultGridHeightCount(6);
static const int32_t maxImageWidthNotResized(1200);
static const uint32_t guiTimerPeriodMs(10);

static const rclcpp::Logger LOGGER = rclcpp::get_logger("spinnaker_ros2");

std::function<void(int)> shutdown_handler;

void signal_handler(int signal)
{
  shutdown_handler(signal);
}

}  // anonymous namespace

namespace spinnaker_ros2
{
bool SpinnakerRos2::initialize(const cli_parameters & parameters)
{
  // Initialize an OpenCV named window
  // @remark It must be called in the main thread
  cv::namedWindow(defaultNodeName, cv::WINDOW_AUTOSIZE);
  cv::waitKey(1);

  // Start pose estimation thread
  std::thread(&SpinnakerRos2::image_process, this).detach();

  // Prepare GUI timer, the purpose is to run GUI code in main thread
  gui_timer_ = create_wall_timer(
    std::chrono::milliseconds(guiTimerPeriodMs),
    std::bind(&SpinnakerRos2::gui_timer_callback, this));
  gui_timer_->cancel();

  // @remark Assume the same grid pattern is used, so grid dimension stays unchanged
  grid_size_.width = static_cast<int>(parameters.dimension.width);
  grid_size_.height = static_cast<int>(parameters.dimension.height);

  cv::FileStorage fs(parameters.calib_xml, cv::FileStorage::READ);
  fs["Intrinsic"] >> camera_intrinsic_;
  fs["Distortion"] >> camera_distortion_;

  // Build chessboard 3D scene
  for (int i = 0; i < grid_size_.height; i++) {
    for (int j = 0; j < grid_size_.width; j++) {
      target_corners_.push_back(cv::Point3f(static_cast<float>(i), static_cast<float>(j), 0.0f));
    }
  }

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

  count++;
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
      // RCLCPP_INFO(LOGGER, "acquired: %d", count);
      if (image_handler) {
        // Call user ligic if it is installed
        // image_handler(image_);
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
  count++;

  // Save the counter for further use, such as sequence ID
  acquired_count_ = count;
#if ENABLE_SENSOR_TOPIC
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
#endif
  // Notify receiving thread data is updated and ready
  // Note lk only unlocks after destructor, there is no unlock/lock as std::unique_lock
  // But it blocks if std::unique_lock/std::conditional_variable still locks
  std::lock_guard<std::mutex> lk(this->frame_ready_mutex_);
  // Note other thread may not get std::conditional_variable::notify_one till this code exits
  frame_ready_condition_.notify_one();
  // Now frame_ready_mutex_ is unlocked
}

/**
 * @brief Object detection and pose estimation
 * @remark This routine runs in a separate thread, and no GUI should be involved
 * @remark It exists when done, so it is instantiated each time an image is acquired
 */
void SpinnakerRos2::pose_detect()
{
  // Backup current sequence #, since it could be changed during the run.
  // Abort the current operation if this happens,
  // which means it takes too long to run, while a new image is acquired
  uint32_t sequence_id = acquired_count_;

  // Generate processing image, the origin image could be changed during the iteration
  cv::Mat gray_image = image_.clone();
  cv::Vec3d eulerAngles;  // object pose in yaw/pitch/roll
  cv::Mat tvec, rvec;     // translation/rotation vector

  // Convert to gray scale
  if ((1 != gray_image.channels()) || (CV_8U != gray_image.type())) {
    cv::cvtColor(gray_image, gray_image, cv::COLOR_BGR2GRAY);
  }

  // Detect chessboard's pose
  std::vector<cv::Point2f> detected_corners;
  bool found = true, solved = false;
  while (true) {
    // The following call may take long time to run
    found = cv::findChessboardCorners(
      gray_image,
      grid_size_,
      detected_corners,
      // It is key to use CALIB_CB_FAST_CHECK
      // to shortcut the detecting when there is no chessboard
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE
    );
    if (!found || pose_detection_exit_ || (sequence_id != acquired_count_)) {
      break;
    }

    cv::cornerSubPix(
      gray_image, detected_corners,
      cv::Size(5, 5),   // half size of serach window
      cv::Size(-1, -1),
      cv::TermCriteria(
        cv::TermCriteria::MAX_ITER +
        cv::TermCriteria::EPS,
        30,   // max number of iterations
        0.1   // min accuracy
      )
    );
    if (pose_detection_exit_ || (sequence_id != acquired_count_)) {
      break;
    }
    if (detected_corners.size() != static_cast<uint32_t>(grid_size_.area())) {
      // Don't reset found, since we still need overlay chessboard corners
      // Note solved is false
      break;
    }

    // It may take long time to detect pose
    solved = cv::solvePnP(
      target_corners_,
      detected_corners,
      camera_intrinsic_,
      camera_distortion_,
      rvec,
      tvec);

    if (pose_detection_exit_ || (sequence_id != acquired_count_)) {
      break;
    }

    if (solved) {
      // rvec and tvec is the transform from model world to camera system
      // say to project a 3D point nose3d to image plane, do:
      // projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix,
      // dist_coeffs, nose_end_point2D);
      // https://learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
      // std::cout << "\trot: " << std::endl << rvec << std::endl;
      // std::cout << "\ttranslate: " << std::endl << tvec << std::endl;
      cv::Mat rotation;
      cv::Rodrigues(rvec, rotation);
      // cv::Affine3d pose(rotation, tvec);
      // pose could be used

      // https://answers.opencv.org/question/161369/retrieve-yaw-pitch-roll-from-rvec/
      // This is the right answer?
      // https://answers.opencv.org/question/16796/computing-attituderoll-pitch-yaw-from-solvepnp/?answer=52913#post-id-52913
      // Now object corners and detected corners are matching up, find pose info
      cv::Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
      double * _r = rotation.ptr<double>();
      double projMatrix[12] = {_r[0], _r[1], _r[2], 0.0,
        _r[3], _r[4], _r[5], 0.0,
        _r[6], _r[7], _r[8], 0.0};
      decomposeProjectionMatrix(
        cv::Mat(3, 4, CV_64FC1, projMatrix),
        cameraMatrix,
        rotMatrix,
        transVect,
        rotMatrixX,
        rotMatrixY,
        rotMatrixZ,
        eulerAngles);
    }

    // Break since we only need to iterate once
    break;
  }

  // Prepare pose text overlay
  if ((!pose_detection_exit_) && (sequence_id == acquired_count_) && (found)) {
    // Critical section to avoid concurrent image manipulation
    // Even there are multiple instances of th sthread at the same time,
    // only one can reach to this point
    std::lock_guard<std::mutex> lk(critical_section_);

    // Draw the corners - when solved is false, lines are in red
    cv::drawChessboardCorners(image_, grid_size_, detected_corners, solved);

    // Pose texts
    if (solved) {
      static char pose_label[128] = {0};
      cv::Point2d label_position(10, image_.rows - 30);

      double * _t = tvec.ptr<double>();
      snprintf(
        pose_label, sizeof(pose_label),
        "translate: %0.2f, %0.2f, %0.2f", _t[0], _t[1], _t[2]);
      cv::putText(
        image_, pose_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        1.0,  // font scale
        cv::Scalar(0, 255, 255),  // color
        2     // thickness
      );

      pose_label[0] = 0;
      label_position.y -= 30;
      snprintf(pose_label, sizeof(pose_label), "roll: %0.2f", eulerAngles[2]);
      cv::putText(
        image_, pose_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        1.0,  // font scale
        cv::Scalar(0, 255, 255),  // color
        2     // thickness
      );

      pose_label[0] = 0;
      label_position.y -= 30;
      snprintf(pose_label, sizeof(pose_label), "pitch: %0.2f", eulerAngles[0]);
      cv::putText(
        image_, pose_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        1.0,  // font scale
        cv::Scalar(0, 255, 255),  // color
        2     // thickness
      );

      pose_label[0] = 0;
      label_position.y -= 30;
      snprintf(pose_label, sizeof(pose_label), "yaw: %0.2f", eulerAngles[1]);
      cv::putText(
        image_, pose_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        1.0,  // font scale
        cv::Scalar(0, 255, 255),  // color
        2     // thickness
      );
    }
    // Release for displaying - this could be 2nd overlay
    gui_timer_->reset();
    // Critical section exits
  } else {
    // if ((!pose_detection_exit_) && (sequence_id == acquired_count_) && (found))
    // Don't care about not found
    if (sequence_id != acquired_count_) {
      RCLCPP_WARN(LOGGER, "pose_detect/rendering_skipped %d/%d", sequence_id, acquired_count_);
    }
  }

  // This thread could be terminated before reaching to this point,
}

/**
 * @brief Image processing
 * @remark This routine runs in a separate thread, and no GUI should be involved
 */
void SpinnakerRos2::image_process()
{
  pose_detection_exit_ = false;
  // Native handle for pose_detect
  // @remark Once a thread is joined or detached, thread id is invalid
  static std::thread::native_handle_type pose_detect_thread_handle = 0;
  uint32_t sequence_id = acquired_count_;
  while (!pose_detection_exit_) {
    // unique_lock instead of lock_guard is used because of conditional_variable's
    // nature which performs lock and unlock during its wait
    std::unique_lock<std::mutex> guard(this->frame_ready_mutex_);
    // The loop's running rate is decided by whoever release frame_ready_condition_
    // ie., frame_ready_condition_.wait unlocks and
    // wait for frame_ready_condition_.notify_one to lock again
    frame_ready_condition_.wait(guard);
    // Now frame_ready_mutex_ is locked again

    // Backup current sequence #, since it could be changed during this loop iteration
    sequence_id = acquired_count_;

    // pose_detection_exit_ could be changed after the above potential long wait,
    // which could be forced to abort, so let's check again
    if (pose_detection_exit_) {
      break;
    }

    // Start object detection and pose estimation thread
    pose_detect_thread_ = std::thread(&SpinnakerRos2::pose_detect, this);
    // Get native handle before detaching. There is potential to use
    // pose_detect_thread_handle via pthread_cancel to terminate currently still running thread
    pose_detect_thread_handle = pose_detect_thread_.native_handle();
    (void)pose_detect_thread_handle;
    pose_detect_thread_.detach();

    // pose_detect thread may not be up yet, but that's OK
    {
      // Critical section to avoid concurrent image manipulation
      std::lock_guard<std::mutex> lk(critical_section_);
      // Annotation
      static char counter_label[128] = {0};
      snprintf(counter_label, sizeof(counter_label), "%d", sequence_id);
      // X/Y, where origin at top left, y+ points down
      cv::Point2d label_position(image_.cols - 150, image_.rows - 30);
      cv::putText(
        image_, counter_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        2.0,  // font scale
        cv::Scalar(0, 255, 0),  // color in BGR
        2     // thickness
      );

      std::chrono::system_clock::time_point current_timestamp{
        std::chrono::system_clock::duration{timestamp_}};

      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_timestamp.time_since_epoch()) % 1000;

      const std::time_t t_c = std::chrono::system_clock::to_time_t(current_timestamp);
      std::stringstream localtime;
      localtime << std::put_time(std::localtime(&t_c), "%F %T");
      localtime << '.' << std::setfill('0') << std::setw(3) << ms.count();
      counter_label[0] = 0;
      label_position.x = 10;
      label_position.y = 20;
      snprintf(counter_label, sizeof(counter_label), "%s", localtime.str().c_str());
      cv::putText(
        image_, counter_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        2.0,  // font scale
        cv::Scalar(0, 0, 255),  // color
        2     // thickness
      );
      // Critical section exits
    }

    // Release for displaying - pose estimation may add another overlay
    gui_timer_->reset();
    // guard is unlocked
  }
}

void SpinnakerRos2::gui_timer_callback()
{
  cv::Mat frame = image_;
  cv::imshow(defaultNodeName, frame);
  cv::waitKey(1);
  gui_timer_->cancel();
}

}  //  namespace spinnaker_ros2

/**
 * @brief Parsing CLI parameters
 *
 * @param argc Count of parameters, including arg name and value pairs
 * @param argv Input paramters
 * @param parameters Output parameters
 * @return true when All good
 * @return false when bad or help so exit
 */
bool argument_parse(int argc, char * argv[], spinnaker_ros2::cli_parameters * parameters)
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

      if ("-x" == *i) {
        try {
          std::string xml = *++i;
          parameters->calib_xml = xml;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad argument, use default calib xml" << parameters->calib_xml << std::endl;
        }
      }

      if ("-w" == *i) {
        try {
          uint32_t width = (uint32_t)std::stoul(*++i);
          parameters->dimension.width = width;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad argument, use default grid width" << parameters->dimension.width << std::endl;
        }
      }

      if ("-h" == *i) {
        try {
          uint32_t height = (uint32_t)std::stoul(*++i);
          parameters->dimension.height = height;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad argument, use default grid height" << parameters->dimension.height << std::endl;
        }
      }

      if ("-l" == *i) {
        try {
          float scale = std::stof(*++i);
          parameters->scale = scale;
        } catch (const std::exception & e) {
          std::cerr << e.what() << std::endl <<
            "Bad argument, use default grid scale" << parameters->scale << std::endl;
        }
      }
    }
  }

  if (help) {
    std::cout << argv[0] << " [-c <camera>] [-x <calib xml>] [--help] " <<
      "[-w <grid width>] [-h <grid height>] [-l <grid dimension scale>]" << std::endl;

    std::cout << "-c (default=" << parameters->camera << ") to specify which camera to use" <<
      std::endl;

    std::cout << "-x (optional, default=" << parameters->calib_xml <<
      ") to specify calibrated data in xml" << std::endl;

    std::cout << "-w (optional, default=" << parameters->dimension.width <<
      ") to specify pattern grid count in width" << std::endl;

    std::cout << "--help showing this help" << std::endl;
    std::cout << "-h (optional, default=" << parameters->dimension.height <<
      ") to specify pattern grid count in height" << std::endl;

    std::cout << "-l (optional, default=" << parameters->scale <<
      ") to specify pattern grid dimension scaling factor" << std::endl;

    return false;
  }

  return true;
}

int main(int argc, char * argv[])
{
  static spinnaker_ros2::cli_parameters parameters = {
    .camera = defaultCamera,
    .scale = defaultGridDimensionScale,
    .calib_xml = defaultCalibrationXml,
    .dimension = {
      .width = defaultGridWithCount,
      .height = defaultGridHeightCount,
    },
  };

  if (!argument_parse(argc, argv, &parameters)) {
    return 0;
  }

  // Verify calib xml's existence
  if (!std::filesystem::exists(parameters.calib_xml)) {
    std::cerr << "Can't find calib xml: " << parameters.calib_xml << std::endl;
    return -1;
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

  if (spinnaker_ros2->initialize(parameters)) {
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
