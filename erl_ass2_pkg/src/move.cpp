// Copyright 2019 Intelligent Robotics Lab
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

#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms)
  {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = now();
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;

    wp.pose.position.x = -7.0;
    wp.pose.position.y = 1.5;
    waypoints_["wp1"] = wp;

    wp.pose.position.x = -3.0;
    wp.pose.position.y = -8.0;
    waypoints_["wp2"] = wp;
    /* cambio cordinate per riconoscimento marker
    wp.pose.position.x = 6.0;
    wp.pose.position.y = 2.0;*/
    wp.pose.position.x = 6.5;
    wp.pose.position.y = 2.2;
    waypoints_["wp3"] = wp;

    wp.pose.position.x =  7.0;
    wp.pose.position.y = -5.0;
    waypoints_["wp4"] = wp;

    wp.pose.position.x = -2.0;
    wp.pose.position.y = -0.4;
    waypoints_["center"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&MoveAction::current_pos_callback, this, _1));
  }

  void current_pos_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(get_logger(), "\n\n [Activating Move] \n\n");
    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(get_logger(), "\n\n [Deactivating Move] \n\n");
    navigation_action_client_->async_cancel_all_goals();
    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void do_work()
  {
    send_feedback(0.0, "Move starting");

    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(),"/navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO_ONCE(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(60));
      is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(15));

      if (!is_action_server_ready) {
        RCLCPP_ERROR_ONCE(get_logger(), "Navigation action server not ready after waiting.");
      }else{
        RCLCPP_INFO_ONCE(get_logger(), "Navigation action server ready and responding.");
      }
    } while (!is_action_server_ready);

    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);
    
    RCLCPP_INFO(get_logger(), "Navigating to: [%s], wp pose [%.2f, %.2f]", wp_to_navigate.c_str(), waypoints_[wp_to_navigate].pose.position.x, waypoints_[wp_to_navigate].pose.position.y);
    RCLCPP_INFO(get_logger(), "cur_pos:(%.2f,%.2f), distance: %.2f", current_pos_.position.x, current_pos_.position.y, dist_to_move);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](NavigationGoalHandle::SharedPtr ,NavigationFeedback feedback) {
      float status = 0.0;
      send_feedback(status,"Move running");
      
      // ckeck stop condition
        if(dist_to_move <= dist_trshold_){
        // abort navigation goal
          RCLCPP_INFO(get_logger(), "Robot inside the target area");
          finish(true,1.0, "Move completed");
      }
    };

    send_goal_options.result_callback = [this](const NavigationGoalHandle::WrappedResult & result) {
      RCLCPP_INFO_ONCE(get_logger(), "Navigation result received");
    };

    future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
  }


  // Dichiarazioni generali per la parte di navigation
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback = const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;
  double dist_trshold_ = 0.4;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}