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
/// April detection and pose estimation as part of
/// ROS2 package for GigEV camera based on Spinnaker SDK
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

#ifndef SPINNAKER_ROS2__APRILTAG_POSE_ESTIMATE_HPP_
#define SPINNAKER_ROS2__APRILTAG_POSE_ESTIMATE_HPP_

#if ENABLE_APRILTAG
extern "C" {
  #include "apriltag/apriltag.h"
}
#endif

// STD
#include <vector>
#include <memory>
#include <string>

#include "spinnaker_ros2/spinnaker_ros2.hpp"
#if ENABLE_APRILTAG
namespace ApriltagPoseEstimate
{
class ApriltagPoseEstimate : public spinnaker_ros2::ImageProcessInstance
{
public:
  ApriltagPoseEstimate(
    spinnaker_ros2::ProcessingSyncPtr sync, std::string family, float size
  );

#pragma mark spinnaker_ros2::ImageProcessInstance
  /**
   * @brief This routine is called from other thread to activate the 'process' thread
   */
  void ready() override;
  void process() override;

protected:
  std::string family_;
  float size_;
  /**
   * @brief 3D grid pattern (chessboard) corners scene with origin at the center
   */
  std::vector<cv::Point3f> target_corners_;
};
typedef std::shared_ptr<ApriltagPoseEstimate> ApriltagPoseEstimatePtr;
}  // namespace ApriltagPoseEstimate
#endif  // ENABLE_APRILTAG
#endif  // SPINNAKER_ROS2__APRILTAG_POSE_ESTIMATE_HPP_
