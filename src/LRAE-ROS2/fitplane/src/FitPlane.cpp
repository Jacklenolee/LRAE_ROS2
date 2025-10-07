/**
 *  Created by Qingchen Bi on 2022/11/05
 */
#include <rclcpp/rclcpp.hpp>
#include "vector"
#include "backward.hpp"
#include <plane.h>

namespace backward
{
backward::SignalHandling sh;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("Traversibility_mapping");
    rclcpp::Rate loop_rate(2);

    FitPlane::World world(0.1, nh);
    float plane_size = 0.3; // 0.5
    FitPlane::PlaneMap planemap(&world, plane_size);
    while(rclcpp::ok())
    {
       rclcpp::spin_some(nh);
       loop_rate.sleep();        
    }
    rclcpp::shutdown();
    return 0;
}
