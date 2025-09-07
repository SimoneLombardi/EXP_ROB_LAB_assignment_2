// end_mission action node

#include <memory>
#include <algorithm>
#include <map>
#include <string>
#include <cstring>
#include <sstream>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class End_Mission : public plansys2::ActionExecutorClient
{
    public:
        End_Mission()
        : plansys2::ActionExecutorClient("finish_mission", 100ms){

            lowest_marker_.first = 100;

            using namespace std::placeholders;
            lowest_marker_sub_ = create_subscription<std_msgs::msg::String>(
                "lowest_marker", 10, std::bind(&End_Mission::lowest_marker_callback, this, std::placeholders::_1));
        }

        void lowest_marker_callback(const std_msgs::msg::String::SharedPtr msg){
            int id;
            double x, y;

            std::istringstream iss(msg->data);
            std::string token;
            std::vector<std::string> parts;

            // split by "_"
            while (std::getline(iss, token, '_')) {
                parts.push_back(token);
            }

            if (parts.size() == 4) {
                id = std::stoi(parts[0]);
                x  = std::stod(parts[1]);
                y  = std::stod(parts[2]);
                mk_cntr_ = std::stoi(parts[3]);
            } else {
                throw std::runtime_error("Invalid marker string: " + msg->data);
            }

            if(id < lowest_marker_.first){
                lowest_marker_.first = id;
                lowest_marker_.second.position.x = x;
                lowest_marker_.second.position.y = y;
                RCLCPP_INFO(get_logger(), "New lowest marker: %d at (%.2f, %.2f)", id, x, y);
            }
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            RCLCPP_INFO(get_logger(), "\n\n [Activating End Mission] \n\n");
            return ActionExecutorClient::on_activate(previous_state);
        }

    private:
        void do_work()
        {
            send_feedback(0.0, "End Mission running");
            if(mk_cntr_ == 4){RCLCPP_INFO(get_logger(), "lowest marker is: %d at (%.2f, %.2f) marker N: %d", lowest_marker_.first, lowest_marker_.second.position.x, lowest_marker_.second.position.y, mk_cntr_);RCLCPP_INFO(get_logger(), "lowest marker is: %d at (%.2f, %.2f) marker N: %d", lowest_marker_.first, lowest_marker_.second.position.x, lowest_marker_.second.position.y, mk_cntr_);
                RCLCPP_INFO(get_logger(), "lowest marker is: %d at (%.2f, %.2f) marker N: %d", lowest_marker_.first, lowest_marker_.second.position.x, lowest_marker_.second.position.y, mk_cntr_);
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

                
                goal_pos_.pose = lowest_marker_.second;
                navigation_goal_.pose = goal_pos_;

                auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

                send_goal_options.feedback_callback = [this](NavigationGoalHandle::SharedPtr ,NavigationFeedback feedback) {
                    float status = 0.0;
                    send_feedback(status,"Move running");
                    if(feedback->distance_remaining < dist_trshold_){
                        RCLCPP_INFO(get_logger(), "Reached goal");
                        finish(true, 1.0, "Mission completed");
                    }
                };

                send_goal_options.result_callback = [this](const NavigationGoalHandle::WrappedResult & result) {
                    RCLCPP_INFO(get_logger(), "Navigation result received");
                    //finish(true, 1.0, "Mission completed");
                };    

                future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
            }
        }


        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lowest_marker_sub_;
        std::pair<int, geometry_msgs::msg::Pose> lowest_marker_;

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
        int mk_cntr_ = 0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<End_Mission>();

    node->set_parameter(rclcpp::Parameter("action_name", "finish_mission"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}