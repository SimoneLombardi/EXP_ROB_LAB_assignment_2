#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "plansys2_msgs/srv/add_problem.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--- arg num [[%d]] ---", argc);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n arg 1 [[%s]] \n\n", argv[1]);

    // create node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("load_problem");

    // create client
    rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedPtr client = node->create_client<plansys2_msgs::srv::AddProblem>("/problem_expert/add_problem");

    // send request
    auto request = std::make_shared<plansys2_msgs::srv::AddProblem::Request>();
    request->problem = argv[1];

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    // wait for the result
    if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %s", result.get()->success ? "true" : "false");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_problem");
    }

    rclcpp::shutdown();
    return 0;
}