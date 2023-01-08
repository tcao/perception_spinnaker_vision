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
/// Chessboard detection and pose estimation as part of
/// ROS2 package for GigEV camera based on Spinnaker SDK
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

#ifndef SPINNAKER_ROS2__CHESSBOARD_POSE_ESTIMATE_HPP_
#define SPINNAKER_ROS2__CHESSBOARD_POSE_ESTIMATE_HPP_

// STD
#include <vector>
#include <memory>

#include "spinnaker_ros2/spinnaker_ros2.hpp"

namespace ChessboardPoseEstimate
{
class ChessboardPoseEstimate : public spinnaker_ros2::ImageProcessInstance
{
public:
  ChessboardPoseEstimate(
    spinnaker_ros2::ProcessingSyncPtr sync,
    uint32_t grid_width, uint32_t grid_height, float scale
  );

#pragma mark spinnaker_ros2::ImageProcessInstance
  /**
   * @brief This routine is called from other thread to activate the 'process' thread
   */
  void ready() override;
  void process() override;

protected:
  cv::Size grid_size_;
  float scale_;
  /**
   * @brief 3D grid pattern (chessboard) corners scene with origin at the center
   */
  std::vector<cv::Point3f> target_corners_;
};
typedef std::shared_ptr<ChessboardPoseEstimate> ChessboardPoseEstimatePtr;
}  // namespace ChessboardPoseEstimate
#endif  // SPINNAKER_ROS2__CHESSBOARD_POSE_ESTIMATE_HPP_
