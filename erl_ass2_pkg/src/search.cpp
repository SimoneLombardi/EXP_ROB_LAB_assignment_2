// search action node
#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <sstream>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "std_msgs/msg/string.hpp"

#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "nav2_msgs/action/spin.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#define MIN_MARKER 11
#define MAX_MARKER 40

using namespace std::chrono_literals;

class Search : public plansys2::ActionExecutorClient
{
    public:
        Search()
        : plansys2::ActionExecutorClient("search", 500ms)
        {
            RCLCPP_INFO(get_logger(), "Creating Search action node");

            using namespace std::placeholders; 
            marker_sub_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&Search::marker_callback, this, _1));  
            if(!marker_sub_){
                RCLCPP_ERROR(get_logger(), "Could not create marker subscription");
            }else{
                RCLCPP_INFO(get_logger(), "Marker subscription created");
            }

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Search::odom_callback, this, _1));
            if(!odom_sub_){
                RCLCPP_ERROR(get_logger(), "Could not create odom subscription");
            }else{
                RCLCPP_INFO(get_logger(), "Odom subscription created");
            }

            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel", 
                rclcpp::QoS(10),
                rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>()
            );
            if(!cmd_vel_pub_){
                RCLCPP_ERROR(get_logger(), "Could not create cmd_vel publisher");
            }else{
                RCLCPP_INFO(get_logger(), "cmd_vel publisher created");
            }

            lowest_marker_pub_ = this->create_publisher<std_msgs::msg::String>(
                "/lowest_marker", 
                rclcpp::QoS(10),
                rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>()
            );
            if(!lowest_marker_pub_){
                RCLCPP_ERROR(get_logger(), "Could not create lowest_marker publisher");
            }else{
                RCLCPP_INFO(get_logger(), "lowest_marker publisher created");   
            }
        }

        void marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
            for (int i =0; i< msg->marker_ids.size(); i++){
                if(msg->marker_ids[i] >= MIN_MARKER && msg->marker_ids[i] <= MAX_MARKER){ // se trovo il marker con id 0, vuol dire che ho trovato quello giusto
                    // add to map
                    if((markers_.insert({msg->marker_ids[i], current_pos_})).second){
                        RCLCPP_INFO(get_logger(), "New Marker ID: %d added to the list", msg->marker_ids[i]);
                        foundMarker_ = true;
                        wp_indx_++;

                        // find lowest marker and publish it
                        auto markers_iter = markers_.begin();
                        std_msgs::msg::String lowest_marker_msg = std_msgs::msg::String();
                        lowest_marker_msg.data = markerToString(markers_iter->first, wp_indx_,markers_iter->second);
                        lowest_marker_pub_->publish(lowest_marker_msg);
                    }
                    
                }else{
                    RCLCPP_WARN(get_logger(), "Marker ID: %d not in the range [%d, %d]", msg->marker_ids[i], MIN_MARKER, MAX_MARKER);
                }
            }
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            current_pos_ = msg->pose.pose;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            RCLCPP_INFO(get_logger(), "\n\n [Activating Search] \n\n");
            cmd_vel_pub_->on_activate();
            lowest_marker_pub_->on_activate();
            return ActionExecutorClient::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            RCLCPP_INFO(get_logger(), "\n\n [Deactivating Search] \n\n");
            cmd_vel_pub_->on_deactivate();
            return ActionExecutorClient::on_deactivate(previous_state);
        }
        

    private:
        void do_work(){
            send_feedback(0.0, "Searching...");
            spin_action_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(shared_from_this(), "/spin");

            // inizializzo il waypoint target per sapere dove mi trovo
            auto wp_at = get_arguments()[1];
            if(strcmp(wp_at.c_str(), "wp1") == 0){
                wp_trgt_ = 1;
                RCLCPP_INFO(get_logger(), "Searching at %s, trg =%d, indx =%d", wp_at.c_str(), wp_trgt_, wp_indx_);
            }else if(strcmp(wp_at.c_str(), "wp2") == 0){
                wp_trgt_ = 2;
                RCLCPP_INFO(get_logger(), "Searching at %s, trg =%d, indx =%d", wp_at.c_str(), wp_trgt_, wp_indx_);
            }else if(strcmp(wp_at.c_str(), "wp3") == 0){
                wp_trgt_ = 3;
                RCLCPP_INFO(get_logger(), "Searching at %s, trg =%d, indx =%d", wp_at.c_str(), wp_trgt_, wp_indx_);
            }else if(strcmp(wp_at.c_str(), "wp4") == 0){
                wp_trgt_ = 4;
                RCLCPP_INFO(get_logger(), "Searching at %s, trg =%d, indx =%d", wp_at.c_str(), wp_trgt_, wp_indx_);
            }else{
                wp_trgt_ = -1;
                RCLCPP_ERROR(get_logger(), "Unknown waypoint: %s", wp_at.c_str());
            }

            if(foundMarker_ && (wp_indx_ == wp_trgt_)){ // se il marker è stato già trovato, posso skippare la ricerca
                // termina pubblicazione su cmd_vel
                geometry_msgs::msg::Twist twist_msg;
                twist_msg.angular.z = 0.0; // stop rotate
                cmd_vel_pub_->publish(twist_msg);

                finish(true, 1.0, "Marker found");
                foundMarker_ = false;
                return;
            }else{  // chiamo l'azione di spin
                geometry_msgs::msg::Twist twist_msg;
                twist_msg.angular.z = 0.7; // rotate at 0.5 rad

                cmd_vel_pub_->publish(twist_msg);
            }
        }

        std::string markerToString(int id, int mk_cntr,const geometry_msgs::msg::Pose& pose){
            std::ostringstream oss;
            oss << id << "_" << pose.position.x << "_" << pose.position.y << "_" << mk_cntr;
            return oss.str();
        }

        // member
        std::map<int, geometry_msgs::msg::Pose> markers_;

        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr lowest_marker_pub_;

        geometry_msgs::msg::Pose current_pos_;
        ros2_aruco_interfaces::msg::ArucoMarkers current_markers_;

        // spin action client
        using SpinGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>;
        using SpinFeedback = const std::shared_ptr<const nav2_msgs::action::Spin::Feedback>;

        rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_action_client_;
        std::shared_future<SpinGoalHandle::SharedPtr> future_spin_goal_handle_;
        SpinGoalHandle::SharedPtr spin_goal_handle_;

        bool foundMarker_ = false;
        int wp_indx_ = 0;
        int wp_trgt_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Search>();

    node->set_parameter(rclcpp::Parameter("action_name", "search"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    //node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}