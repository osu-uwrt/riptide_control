#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
        list_parameters_cli = this->create_client<rcl_interfaces::srv::ListParameters>("/talos/autonomy/list_parameters");

    }

    void callService()
    {
        auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
        auto result = list_parameters_cli->async_send_request(request);

        auto a = this->shared_from_this();

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got Parameters List!");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        }
    }


  private:
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_cli;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  node->callService();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
