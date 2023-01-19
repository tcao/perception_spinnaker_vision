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
/// Chessboard detection and pose estimation as part of
/// ROS2 package for GigEV camera based on Spinnaker SDK
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

// STD
#include <vector>

#include "spinnaker_ros2/chessboard_pose_estimate.hpp"

namespace ChessboardPoseEstimate
{
static const rclcpp::Logger ChessboardLogger = rclcpp::get_logger("chessboard_processor");
static const int processorID(1);

ChessboardPoseEstimate::ChessboardPoseEstimate(
  spinnaker_ros2::ProcessingSyncPtr sync,
  uint32_t grid_width, uint32_t grid_height, float scale)
: ImageProcessInstance(sync),
  scale_(scale)
{
  image_process_context_.ready_ = false;
  image_process_context_.processing_exit_ = false;

  // @remark Assume the same grid pattern is used, so grid dimension stays unchanged
  grid_size_.width = static_cast<int>(grid_width);
  grid_size_.height = static_cast<int>(grid_height);

  // Build chessboard 3D scene
  for (int i = 0; i < grid_size_.height; i++) {
    for (int j = 0; j < grid_size_.width; j++) {
      target_corners_.push_back(
        cv::Point3f(
          static_cast<float>(i) * scale_,
          static_cast<float>(j) * scale_, 0.0f));
    }
  }

  RCLCPP_INFO(ChessboardLogger, "chessboard processor instantiated");
}

/**
 * @brief This routine is called from other thread to activate the 'process' thread
 */
void ChessboardPoseEstimate::ready()
{
  // @case 1 - it is possible the following call is blocking
  std::lock_guard<std::mutex> lk(image_process_context_.processing_ready_mutex_);
  image_process_context_.processing_ready_condition_.notify_one();
  image_process_context_.ready_ = true;
}

void ChessboardPoseEstimate::process()
{
  image_process_context_.processing_exit_ = false;
  uint32_t sequence = 0;

  sync_->get_camera_parameters(
    image_process_context_.camera_intrinsic_, image_process_context_.camera_distortion_);


  cv::Vec3d eulerAngles;  // object pose in yaw/pitch/roll
  cv::Mat tvec, rvec;     // translation/rotation vector

  RCLCPP_INFO(ChessboardLogger, "chessboard processor thread is up");

  static uint32_t count = 0;
  while (!image_process_context_.processing_exit_) {
    count++;
    RCLCPP_INFO(ChessboardLogger, "chessboard ready: %d", sync_->get_sequence_number());
    std::unique_lock<std::mutex> guard(image_process_context_.processing_ready_mutex_);
    image_process_context_.processing_ready_condition_.wait(guard);
    RCLCPP_INFO(ChessboardLogger, "chessboard run: %d", sync_->get_sequence_number());
    spinnaker_ros2::RaiiSync submit_sync(sync_, processorID, sync_->get_sequence_number());

    // Now notification is received and the mutex is locked
    if (image_process_context_.processing_exit_) {
      break;
    }
    // Acknowledge notification, need track the flag since it will be set by other thread
    image_process_context_.ready_ = false;

    // Since the following code could be lengthy, let's unlock
    // such at it won't block further notification from other thread, ie.,
    // it is used to address the above @case 1
    guard.unlock();

    sequence = sync_->get_sequence_number();

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

    // Detect chessboard's pose
    std::vector<cv::Point2f> detected_corners;
    bool found = cv::findChessboardCorners(
      gray_image,
      grid_size_,
      detected_corners,
      // It is key to use CALIB_CB_FAST_CHECK
      // to shortcut the detecting when there is no chessboard
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE
    );
    RCLCPP_INFO(ChessboardLogger, "detected: %d", found);

    if (image_process_context_.processing_exit_) {
      break;
    }

    // Prepare for next run when one of the following is true:
    // 1 - no chessboard is found
    // 2 - new acquisition received
    if (!found || image_process_context_.ready_) {
      image_process_context_.ready_ = false;
      continue;
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
    RCLCPP_INFO(ChessboardLogger, "subpix");

    if (image_process_context_.processing_exit_) {
      break;
    }
    if (image_process_context_.ready_) {
      image_process_context_.ready_ = false;
      continue;
    }

    // It may take long time to detect pose
    bool solved = false;
    try {
      solved = cv::solvePnP(
        target_corners_,
        detected_corners,
        image_process_context_.camera_intrinsic_,
        image_process_context_.camera_distortion_,
        rvec,
        tvec);
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(ChessboardLogger, "solvePnP exception: %d\n%s", count, e.what());
      if (image_process_context_.ready_) {
        image_process_context_.ready_ = false;
      }
      continue;
    }
    RCLCPP_INFO(ChessboardLogger, "solved: %d", solved);

    if (image_process_context_.processing_exit_) {
      break;
    }

    if (image_process_context_.ready_) {
      image_process_context_.ready_ = false;
      continue;
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

      if (image_process_context_.processing_exit_) {
        break;
      }

      if (image_process_context_.ready_) {
        image_process_context_.ready_ = false;
        continue;
      }
    }

    RCLCPP_INFO(ChessboardLogger, "enter cs: %p", image.data);

    // Ready for overlay composing
    sync_->enter_critical_section(true);

    // Draw the corners - when solved is false, lines are in red
    cv::drawChessboardCorners(image, grid_size_, detected_corners, solved);

    // Draw pose info if sovled
    if (solved) {
      static char pose_label[128] = {0};
      pose_label[0] = 0;
      cv::Point2d label_position(10, image.rows - 30);

      double * _t = tvec.ptr<double>();
      snprintf(
        pose_label, sizeof(pose_label),
        "translate: %0.2f, %0.2f, %0.2f", _t[0], _t[1], _t[2]);
      cv::putText(
        image, pose_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        1.0,  // font scale
        cv::Scalar(0, 255, 255),  // Yellow
        2     // thickness
      );

      pose_label[0] = 0;
      label_position.y -= 30;
      snprintf(pose_label, sizeof(pose_label), "roll: %0.2f", eulerAngles[2]);
      cv::putText(
        image, pose_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        1.0,  // font scale
        cv::Scalar(0, 255, 255),  // Yellow
        2     // thickness
      );

      pose_label[0] = 0;
      label_position.y -= 30;
      snprintf(pose_label, sizeof(pose_label), "pitch: %0.2f", eulerAngles[0]);
      cv::putText(
        image, pose_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        1.0,  // font scale
        cv::Scalar(0, 255, 255),  // Yellow
        2     // thickness
      );

      pose_label[0] = 0;
      label_position.y -= 30;
      snprintf(pose_label, sizeof(pose_label), "yaw: %0.2f", eulerAngles[1]);
      cv::putText(
        image, pose_label, label_position,
        cv::FONT_HERSHEY_PLAIN,
        1.0,  // font scale
        cv::Scalar(0, 255, 255),  // color
        2     // thickness
      );
    }
    sync_->enter_critical_section(false);
    RCLCPP_INFO(ChessboardLogger, "exit cs");
  }   // while
}
}  // namespace ChessboardPoseEstimate
