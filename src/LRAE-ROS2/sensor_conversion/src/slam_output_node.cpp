//
// Created by hjl on 2021/9/18.
//

#include "slam_simulation/slam_output.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("slam_sim_output");
    SlamOutput slam_out_put(nh);

    rclcpp::spin_some(nh);
    return 0;
}
