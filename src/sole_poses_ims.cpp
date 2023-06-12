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

#include "sole_poses_ims/sole_poses_ims.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

namespace sole_poses_ims
{

SolePosesIms::SolePosesIms(const rclcpp::NodeOptions & options)
: rclcpp::Node{"sole_poses_ims", options}
{
  // Parameters
  float l_sole_default_x = this->declare_parameter("l_sole_default_x", 0.0);
  float l_sole_default_y = this->declare_parameter("l_sole_default_y", 0.0);
  float l_sole_default_z = this->declare_parameter("l_sole_default_z", 0.0);
  float r_sole_default_x = this->declare_parameter("r_sole_default_x", 0.0);
  float r_sole_default_y = this->declare_parameter("r_sole_default_y", 0.0);
  float r_sole_default_z = this->declare_parameter("r_sole_default_z", 0.0);
  std::string frame_id = this->declare_parameter("frame_id", "base_link");

  RCLCPP_DEBUG(get_logger(), "Parameters: ");
  RCLCPP_DEBUG(get_logger(), "  l_sole_default_x : %f", l_sole_default_x);
  RCLCPP_DEBUG(get_logger(), "  l_sole_default_y : %f", l_sole_default_y);
  RCLCPP_DEBUG(get_logger(), "  l_sole_default_z : %f", l_sole_default_z);
  RCLCPP_DEBUG(get_logger(), "  r_sole_default_x : %f", r_sole_default_x);
  RCLCPP_DEBUG(get_logger(), "  r_sole_default_y : %f", r_sole_default_y);
  RCLCPP_DEBUG(get_logger(), "  r_sole_default_z : %f", r_sole_default_z);

  // Publishers
  solePosesPub = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);

  server = std::make_shared<interactive_markers::InteractiveMarkerServer>("sole_poses_ims", this);

  solePoses.l_sole.position.x = l_sole_default_x;
  solePoses.l_sole.position.y = l_sole_default_y;
  solePoses.l_sole.position.z = l_sole_default_z;
  solePoses.r_sole.position.x = r_sole_default_x;
  solePoses.r_sole.position.y = r_sole_default_y;
  solePoses.r_sole.position.z = r_sole_default_z;

  auto lFootInteractiveMarker = createMarker("l_foot", solePoses.l_sole, frame_id);
  auto rFootInteractiveMarker = createMarker("r_foot", solePoses.r_sole, frame_id);

  server->insert(
    lFootInteractiveMarker,
    std::bind(&SolePosesIms::lFootCallback, this, std::placeholders::_1));
  server->insert(
    rFootInteractiveMarker,
    std::bind(&SolePosesIms::rFootCallback, this, std::placeholders::_1));

  // 'commit' changes and send to all clients
  server->applyChanges();

  solePosesPub->publish(solePoses);
}

visualization_msgs::msg::InteractiveMarker SolePosesIms::createMarker(
  const std::string & name,
  const geometry_msgs::msg::Pose & pose,
  const std::string & frame_id)
{
  visualization_msgs::msg::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = frame_id;
  interactive_marker.header.stamp = get_clock()->now();
  interactive_marker.name = name;
  interactive_marker.scale = 0.15;
  interactive_marker.pose = pose;

  // create a non-interactive control which contains the box
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);

  return interactive_marker;
}

void SolePosesIms::lFootCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  solePoses.l_sole = feedback->pose;
  publishSolePoses();
}

void SolePosesIms::rFootCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  solePoses.r_sole = feedback->pose;
  publishSolePoses();
}

void SolePosesIms::publishSolePoses()
{
  solePosesPub->publish(solePoses);
}

}  // namespace sole_poses_ims

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sole_poses_ims::SolePosesIms)
