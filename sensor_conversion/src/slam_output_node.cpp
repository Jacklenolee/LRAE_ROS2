#include "../include/slam_simulation/slam_output.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamOutput>());
    rclcpp::shutdown();
    return 0;
}
