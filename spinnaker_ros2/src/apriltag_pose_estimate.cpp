//
// Copyright 2023 Ting Cao <cao_ting@yahoo.com>
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
/// Apriltag detection and pose estimation as part of
/// ROS2 package for GigEV camera based on Spinnaker SDK
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

// STD
#include <string>

// apriltag
#if ENABLE_APRILTAG
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#endif

#include "spinnaker_ros2/apriltag_pose_estimate.hpp"

#if ENABLE_APRILTAG
extern "C" {
typedef apriltag_family_t * (* tag_family_create_t)(void);
typedef void (* tag_family_destroy_t)(apriltag_family_t *);
}

namespace ApriltagPoseEstimate
{
struct apriltag_family_entry
{
  std::string family_;
  tag_family_create_t create;
  tag_family_destroy_t destroy;
};

apriltag_family_entry apriltag_family_entries[] = {
  {
    .family_ = "tag36h11",
    .create = tag36h11_create,
    .destroy = tag36h11_destroy,
  },
  {
    .family_ = "tagStandard41h12",
    .create = tagStandard41h12_create,
    .destroy = tagStandard41h12_destroy,
  },
};

static const rclcpp::Logger ApriltagLogger = rclcpp::get_logger("apriltag_processor");
static const int processorID(2);

ApriltagPoseEstimate::ApriltagPoseEstimate(
  spinnaker_ros2::ProcessingSyncPtr sync, std::string family, float size)
: ImageProcessInstance(sync),
  family_(family),
  size_(size)
{
  image_process_context_.ready_ = false;
  image_process_context_.processing_exit_ = false;

  RCLCPP_INFO(ApriltagLogger, "apriltag processor instantiated");
}

/**
 * @brief This routine is called from other thread to activate the 'process' thread
 */
void ApriltagPoseEstimate::ready()
{
  std::lock_guard<std::mutex> lk(image_process_context_.processing_ready_mutex_);
  image_process_context_.processing_ready_condition_.notify_one();
  image_process_context_.ready_ = true;
}

void ApriltagPoseEstimate::process()
{
  image_process_context_.processing_exit_ = false;
  RCLCPP_INFO(ApriltagLogger, "apriltag processor thread is up");

  sync_->get_camera_parameters(
    image_process_context_.camera_intrinsic_, image_process_context_.camera_distortion_);

  apriltag_family_t * tag_family = nullptr;
  apriltag_family_entry * tag_family_entry = nullptr;
  for (size_t i = 0; i < sizeof(apriltag_family_entries) / sizeof(apriltag_family_entries[0]);
    i++)
  {
    apriltag_family_entry * entry = &apriltag_family_entries[i];
    if (entry->family_ == family_) {
      tag_family_entry = entry;
      break;
    }
  }

  if (nullptr == tag_family_entry) {
    RCLCPP_ERROR(ApriltagLogger, "unable to find apriltag family supported: %s", family_.c_str());
    return;
  }

  tag_family = tag_family_entry->create();
  apriltag_detector_t * tag_detector = apriltag_detector_create();
  errno = 0;
  apriltag_detector_add_family(tag_detector, tag_family);
  if (errno == ENOMEM) {
    RCLCPP_ERROR(
      ApriltagLogger,
      "unable to add apriltag family %s, quiting",
      tag_family_entry->family_.c_str());
    tag_family_entry->destroy(tag_family);
    return;
  }
  // Now detector is setup OK with default parameters

  static uint32_t count = 0;
  while (!image_process_context_.processing_exit_) {
    count++;
    RCLCPP_INFO(
      ApriltagLogger,
      "apriltag ready: %d", sync_->get_sequence_number());
    std::unique_lock<std::mutex> guard(image_process_context_.processing_ready_mutex_);
    image_process_context_.processing_ready_condition_.wait(guard);
    RCLCPP_INFO(
      ApriltagLogger,
      "apriltag run: %d", sync_->get_sequence_number());
    spinnaker_ros2::RaiiSync submit_sync(sync_, processorID, sync_->get_sequence_number());
    // Now notification is received and the mutex is locked
    if (image_process_context_.processing_exit_) {
      break;
    }
    // Acknowledge notification, need track the flag since it will be set by other thread
    image_process_context_.ready_ = false;

    // Allow ready is called for next run before this run finishes
    guard.unlock();

    // Get a frame reference, and it could be changed during the process
    cv::Mat image = sync_->get_image();
    // Keep it and may need it for composing overlay

    // Generate a gray level image
    // And this won't change when a furth acquisition is done, but it should be abandoned
    cv::Mat gray_image = image.clone();
    // Convert to gray scale
    if ((1 != gray_image.channels()) || (CV_8U != gray_image.type())) {
      cv::cvtColor(gray_image, gray_image, cv::COLOR_BGR2GRAY);
    }

    // Fill in image_data for the Mat data
    // @remark Don't understand why it declares constants, this prevents optimization
    image_u8_t image_data = {
      .width = gray_image.cols,
      .height = gray_image.rows,
      .stride = gray_image.cols,
      .buf = gray_image.data,
    };

    RCLCPP_INFO(ApriltagLogger, "apriltag detecting");
    // Perform detection
    errno = 0;
    zarray_t * detections = apriltag_detector_detect(tag_detector, &image_data);
    // TODO(tcao): Check EAGAIN only?
    if (0 != errno) {
      // TODO(tcao): Confirm if apriltag_detections_destroy should be called
      RCLCPP_ERROR(ApriltagLogger, "detecting ends with errno: %d", errno);
      continue;
    }

    if (image_process_context_.processing_exit_) {
      apriltag_detections_destroy(detections);
      break;
    }

    // Prepare for next run when new acquisition received
    if (image_process_context_.ready_) {
      image_process_context_.ready_ = false;
      RCLCPP_WARN(
        ApriltagLogger,
        "skipped after detecting %d at loop %d", zarray_size(detections), count);
      apriltag_detections_destroy(detections);
      continue;
    }

    // Ready for overlay composing
    RCLCPP_INFO(ApriltagLogger, "detected tag #: %d", zarray_size(detections));
    sync_->enter_critical_section(true);
    RCLCPP_INFO(ApriltagLogger, "enter cs: %p", image.data);
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t * det;
      zarray_get(detections, i, &det);
      cv::line(
        image,
        cv::Point(det->p[0][0], det->p[0][1]),
        cv::Point(det->p[1][0], det->p[1][1]),
        cv::Scalar(0, 0xff, 0), 2);             // Red
      cv::line(
        image,
        cv::Point(det->p[0][0], det->p[0][1]),
        cv::Point(det->p[3][0], det->p[3][1]),
        cv::Scalar(0, 0, 0xff), 2);             // Green
      cv::line(
        image,
        cv::Point(det->p[1][0], det->p[1][1]),
        cv::Point(det->p[2][0], det->p[2][1]),
        cv::Scalar(0xff, 0, 0), 2);             // Blue
      cv::line(
        image,
        cv::Point(det->p[2][0], det->p[2][1]),
        cv::Point(det->p[3][0], det->p[3][1]),
        cv::Scalar(0, 0xff, 0xff), 2);          // Yellow

      static char id_text[8] = {0};
      id_text[0] = 0;
      snprintf(
        id_text, sizeof(id_text),
        "%d", det->id);

      int baseline = 0;
      cv::Size text_size = cv::getTextSize(
        id_text,
        cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
        1.0,
        2,
        &baseline);
      cv::putText(
        image,
        id_text,
        cv::Point(det->c[0] - text_size.width / 2, det->c[1] + text_size.height / 2),
        cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0,
        cv::Scalar(0xff, 0xff, 0xff),
        2);
    }
    RCLCPP_INFO(ApriltagLogger, "detected tags processed");
    sync_->enter_critical_section(false);
    RCLCPP_INFO(ApriltagLogger, "exit cs");

    apriltag_detections_destroy(detections);
  }

  apriltag_detector_destroy(tag_detector);
  tag_family_entry->destroy(tag_family);
}

}  // // namespace ApriltagPoseEstimate
#endif  // ENABLE_APRILTAG
