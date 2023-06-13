// Copyright 2023 Kenji Brameld
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

#ifndef SOLE_POSES_IMS__SOLE_POSES_IMS_HPP_
#define SOLE_POSES_IMS__SOLE_POSES_IMS_HPP_

#include <memory>
#include <string>

#include "biped_interfaces/msg/sole_poses.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/node.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"

namespace sole_poses_ims
{

class SolePosesIms : public rclcpp::Node
{
public:
  explicit SolePosesIms(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  rclcpp::Publisher<biped_interfaces::msg::SolePoses>::SharedPtr solePosesPub;

  visualization_msgs::msg::InteractiveMarker createMarker(
    const std::string & name,
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame_id);

  void lFootCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
  void rFootCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  void publishSolePoses();

  biped_interfaces::msg::SolePoses solePoses;
};

}  // namespace sole_poses_ims

#endif  // SOLE_POSES_IMS__SOLE_POSES_IMS_HPP_
