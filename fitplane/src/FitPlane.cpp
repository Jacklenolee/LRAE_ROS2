/**
 *  Created by Qingchen Bi on 2022/11/05
 */
#include <rclcpp/rclcpp.hpp>
#include "vector"
#include "backward.hpp"
#include "../include/plane.h"
#include "../include/World.h"

namespace backward
{
backward::SignalHandling sh;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("Traversibility_mapping");
    rclcpp::Rate loop_rate(2);

    FitPlane::World world(0.1, nh);   // 创建World对象,分辨率为0.1米
    float plane_size = 0.3; // 0.5    // 设置平面大小为0.3米
    FitPlane::PlaneMap planemap(&world, plane_size);  // 创建PlaneMap对象,并传入world和plane_size
    while(rclcpp::ok())
    {
       rclcpp::spin_some(nh);
       loop_rate.sleep();        
    }
    return 0;
}
