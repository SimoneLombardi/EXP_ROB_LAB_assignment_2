// find_marker action node

#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class Find_Marker : public plansys2::ActionExecutorClient
{
    public:
        Find_Marker()
        : plansys2::ActionExecutorClient("find_marker", 100ms){
            progress_ = 1.0;
        }

    private:
        void do_work()
        {
            finish(true, progress_, "marker found");
        }

        float progress_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Find_Marker>();

    node->set_parameter(rclcpp::Parameter("action_name", "find_marker"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}