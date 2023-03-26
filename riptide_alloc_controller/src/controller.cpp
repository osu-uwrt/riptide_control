#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "riptide_alloc_controller/allocated_controller.hpp"


int main(int argc, char ** argv)
{
  // init ros node
  rclcpp::init(argc, argv);

  // Create the node and spin it
  auto node = std::make_shared<riptide_alloc_controller::AllocController>();
  rclcpp::spin(node);

  // the node has ben called to shutdown, so rclcpp needs to be shut down as well
  rclcpp::shutdown();
}
