/**
 *  用于合并探索地图的ROS2节点,主要功能是将局部占据栅格地图合并成全局地图。
 */
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "Eigen/Eigen"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//CONTXY2DISC: 将连续坐标转换为离散网格索引
//DISCXY2CONT: 将离散网格索引转换为连续坐标(网格中心点)
#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

//mapUpdateData结构体: 存储机器人坐标和更新状态
struct mapUpdateData
{
    Eigen::Vector3d robot_coord;
   //机器人坐标
    bool updated = false;
//是否更新
}; 
mapUpdateData** mapUpdate = NULL;
 //地图更新数据
nav_msgs::msg::OccupancyGrid mapData;
//局部地图数据
nav_msgs::msg::OccupancyGrid globalMapData;
 //全局地图数据
bool hasMap = false;
 //是否存在地图
bool initSucessed = false;
 //是否初始化成功

geometry_msgs::msg::Point ego_position_;
 //当前机器人位置
geometry_msgs::msg::Point ego_position_last;
 //上一次机器人位置

float mapUpdateRes;
//地图更新分辨率
int mapUpdateWidth;
 //地图更新宽度
int mapUpdateHeight;
 //地图更新高度

int map_w = 1800;
 //地图宽度
int map_h = 1800;
 //地图高度
double mapinitox = -90.0;
 //地图初始x坐标
double mapinitoy = -90.0;
 //地图初始y坐标
double merge_size = 10.0;
 //合并大小
double safe_obs_dis = 20.0;
//安全障碍物距离

std_msgs::msg::Int8 isMoveInited;
 //是否移动初始化  

//地图回调函数: 接收局部地图数据
void mapCallBack(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    mapData=*msg;
    hasMap = true;
}

//在全局地图边界设置障碍物。
void SetBround(nav_msgs::msg::OccupancyGrid& globalMapData)
{
    //遍历地图的每一行。
    for(int i = 0; i < globalMapData.info.height; i++)
    {
        //每一行的第一个元素（左边界）设置为 100，表示该位置是障碍物
        globalMapData.data[0 + i * globalMapData.info.width] = 100;  
        //每一行的最后一个元素（右边界）设置为 100，表示该位置是障碍物。
        globalMapData.data[globalMapData.info.width - 1 + i * globalMapData.info.width] = 100;  
    }
    //遍历地图的每一列
    for(int j = 0; j < globalMapData.info.width; j++)
    {
        //将每一列的第一个元素（上边界）设置为 100，表示该位置是障碍物。
        globalMapData.data[j + 0 * globalMapData.info.width] = 100;
        //将每一列的最后一个元素（下边界）设置为 100，表示该位置是障碍物。
        globalMapData.data[((globalMapData.info.width - 1) + (globalMapData.info.height - 1)  * globalMapData.info.width) - j] = 100;
    }
}

//初始化全局地图,初始化全局地图的尺寸、分辨率等参数。
void initGlobalMap()
{
    globalMapData.header.frame_id = mapData.header.frame_id;
    globalMapData.header.stamp = mapData.header.stamp;
    globalMapData.info.origin.position.x = mapinitox;
    globalMapData.info.origin.position.y = mapinitoy;
    globalMapData.info.origin.position.z = -0.5;

    globalMapData.info.resolution = mapData.info.resolution;
    globalMapData.info.width = map_w; // globalMapData.info.origin.position.x * (-2) / globalMapData.info.resolution;
    globalMapData.info.height = map_h; //globalMapData.info.origin.position.y * (-2) / globalMapData.info.resolution;
    
    //这行代码创建一个大小为全局地图宽度乘以高度的int8_t类型的向量，并初始化所有元素为-1。
    std::vector<int8_t> tmp(globalMapData.info.width * globalMapData.info.height, -1);
    globalMapData.data = tmp;

    // 初始化一个二维数组mapUpdate，用于存储地图更新数据。
    mapUpdateRes = mapData.info.resolution * 2;
    mapUpdateWidth = (int)(globalMapData.info.width * mapData.info.resolution / mapUpdateRes);
    mapUpdateHeight = (int)(globalMapData.info.height * mapData.info.resolution / mapUpdateRes);
    mapUpdate = new mapUpdateData*[mapUpdateWidth];
    for(int i = 0; i < mapUpdateWidth; i++)
    {
        mapUpdate[i] = new mapUpdateData[mapUpdateHeight];
    }
    
   //在全局地图边界设置障碍物。上下左右边界设置为障碍物
    SetBround(globalMapData);
    initSucessed = true;
}

//地图合并: 将局部地图数据合并到全局地图中。  
//计算机器人位置对应的网格索引
//遍历局部地图的每个网格
//根据距离阈值和障碍物状态进行地图更新
//使用加权平均方式合并地图数据  
void mapMerge()
{
    int rx = mapinitox;
    int ry = mapinitox;
    //如果机器人位置发生变化,则更新机器人位置对应的网格索引
    if(ego_position_last.x != mapinitox)
    {
        rx = CONTXY2DISC(ego_position_last.x - globalMapData.info.origin.position.x, mapUpdateRes); 
        ry = CONTXY2DISC(ego_position_last.y - globalMapData.info.origin.position.y, mapUpdateRes);
    }

    for(int i = 0; i < mapData.info.width; i++)
    {
        for(int j = 0; j < mapData.info.height; j++)
        {
            double px = DISCXY2CONT(i, mapData.info.resolution) + mapData.info.origin.position.x;
            double py = DISCXY2CONT(j, mapData.info.resolution) + mapData.info.origin.position.y;
            if(abs(px - ego_position_.x) > merge_size || abs(py - ego_position_.y) > merge_size)
                continue;
            else
            {
                int ix = CONTXY2DISC(px - globalMapData.info.origin.position.x, globalMapData.info.resolution); 
                int iy = CONTXY2DISC(py - globalMapData.info.origin.position.y, globalMapData.info.resolution);

                int ux = CONTXY2DISC(px - globalMapData.info.origin.position.x, mapUpdateRes); 
                int uy = CONTXY2DISC(py - globalMapData.info.origin.position.y, mapUpdateRes);

                if(rx == ux && ry == uy && ego_position_last.x != mapinitox)
                {
                    if(mapUpdate[rx][ry].updated)
                        continue;
                    else
                    {
                        if(globalMapData.data[ix + iy * globalMapData.info.width] == -1)
                            globalMapData.data[ix + iy * globalMapData.info.width] = mapData.data[i + j * mapData.info.width];
                        else if(globalMapData.data[ix + iy * globalMapData.info.width] < 90 && globalMapData.data[ix + iy * globalMapData.info.width] >=0)
                        {
                            if(mapData.data[i + j * mapData.info.width] != -1)
                                globalMapData.data[ix + iy * globalMapData.info.width] = 
                                    (int)(0.2 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                    0.8 * (double)(mapData.data[i + j * mapData.info.width]));
                        }
                        else
                        {
                            if(abs(px - ego_position_.x) >= safe_obs_dis || abs(py - ego_position_.y) >= safe_obs_dis)
                            {
                                if(mapData.data[i + j * mapData.info.width] < 119 && globalMapData.data[ix + iy * globalMapData.info.width] < 119) 
                                {
                                    if(mapData.data[i + j * mapData.info.width] != -1)
                                        globalMapData.data[ix + iy * globalMapData.info.width] = 
                                            (int)(0.2 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                            0.8 * (double)(mapData.data[i + j * mapData.info.width]));
                                }
                                else
                                    globalMapData.data[ix + iy * globalMapData.info.width] = 100;
                            }
                            else
                            {
                                if(mapData.data[i + j * mapData.info.width] < 119 && globalMapData.data[ix + iy * globalMapData.info.width] < 119) 
                                {
                                    if(mapData.data[i + j * mapData.info.width] != -1)
                                        globalMapData.data[ix + iy * globalMapData.info.width] = 
                                            (int)(0.96 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                            0.04 * (double)(mapData.data[i + j * mapData.info.width]));
                                }
                                else
                                    globalMapData.data[ix + iy * globalMapData.info.width] = 100;
                            }
                        }
                        mapUpdate[rx][ry].updated = true;
                    }
                }
                else
                {
                    if(mapUpdate[ux][uy].updated)
                        continue;
                    else
                    {
                        if(globalMapData.data[ix + iy * globalMapData.info.width] == -1)
                            globalMapData.data[ix + iy * globalMapData.info.width] = mapData.data[i + j * mapData.info.width];
                        else if(globalMapData.data[ix + iy * globalMapData.info.width] < 90 && globalMapData.data[ix + iy * globalMapData.info.width] >=0)
                        {
                            if(mapData.data[i + j * mapData.info.width] != -1)
                                globalMapData.data[ix + iy * globalMapData.info.width] = 
                                    (int)(0.2 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                    0.8 * (double)(mapData.data[i + j * mapData.info.width]));
                        }
                        else
                        {
                            if(abs(px - ego_position_.x) >= safe_obs_dis || abs(py - ego_position_.y) >= safe_obs_dis)
                            {
                                if(mapData.data[i + j * mapData.info.width] < 119 && globalMapData.data[ix + iy * globalMapData.info.width] < 119) 
                                {
                                    if(mapData.data[i + j * mapData.info.width] != -1)
                                        globalMapData.data[ix + iy * globalMapData.info.width] = 
                                            (int)(0.2 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                            0.8 * (double)(mapData.data[i + j * mapData.info.width]));
                                }
                                else
                                    globalMapData.data[ix + iy * globalMapData.info.width] = 100;
                            }
                            else
                            {
                                if(mapData.data[i + j * mapData.info.width] < 119 && globalMapData.data[ix + iy * globalMapData.info.width] < 119) 
                                {
                                    if(mapData.data[i + j * mapData.info.width] != -1)
                                        globalMapData.data[ix + iy * globalMapData.info.width] = 
                                            (int)(0.96 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                            0.04 * (double)(mapData.data[i + j * mapData.info.width]));
                                }
                                else
                                    globalMapData.data[ix + iy * globalMapData.info.width] = 100;
                            }
                        }
                    }
                }
            }                                         
        }
    }
} 

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node_handle = rclcpp::Node::make_shared("exploration_map_merge");

    node_handle->declare_parameter<int>("map_w", map_w); //设置全局地图宽度
    node_handle->declare_parameter<int>("map_h", map_h); //设置全局地图高度
    node_handle->declare_parameter<double>("mapinitox", mapinitox); //设置全局地图初始x坐标
    node_handle->declare_parameter<double>("mapinitoy", mapinitoy); //设置地全局图初始y坐标
    node_handle->declare_parameter<double>("merge_size", merge_size); //地图合并的有效范围
    node_handle->declare_parameter<double>("safe_obs_dis", safe_obs_dis); //设置安全障碍物距离
    
    node_handle->get_parameter("map_w", map_w);
    node_handle->get_parameter("map_h", map_h);
    node_handle->get_parameter("mapinitox", mapinitox);
    node_handle->get_parameter("mapinitoy", mapinitoy);
    node_handle->get_parameter("merge_size", merge_size);
    node_handle->get_parameter("safe_obs_dis", safe_obs_dis);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub = node_handle->create_subscription<nav_msgs::msg::OccupancyGrid>("/plane_OccMap", 1000, mapCallBack);
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr globalMap_pub = node_handle->create_publisher<nav_msgs::msg::OccupancyGrid>("/globalMap", 10);
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr move_inited_pub = node_handle->create_publisher<std_msgs::msg::Int8>("/MoveInited", 1);
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_position_pub = node_handle->create_publisher<geometry_msgs::msg::Point>("/RobotPosition", 1);
    // ros::Publisher trav_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("local_traversibility_ponit_cloud", 1);

    //地图坐标跟随机器人位置坐标
    ego_position_last.x = mapinitox;
    ego_position_last.y = mapinitox;
    isMoveInited.data = 0;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_handle->get_clock());
    std::shared_ptr< tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    rclcpp::WallRate rate(2); // 2 Hz
    while(rclcpp::ok())
    {
        //如果存在地图且未初始化,则初始化全局地图
        if(hasMap && !initSucessed)
        {
            initGlobalMap();
        }

        //获取当前机器人位置
        geometry_msgs::msg::Pose current_pose_ros;
        geometry_msgs::msg::TransformStamped transform_pose;
        tf2::Quaternion quat; 
        double roll,pitch,yaw;
        try
        {
            transform_pose = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            ego_position_.x = transform_pose.transform.translation.x;
            ego_position_.y = transform_pose.transform.translation.y;
            tf2::fromMsg(transform_pose.transform.rotation,quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);

            ego_position_.z = transform_pose.transform.translation.z;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_handle->get_logger(), "%s", ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        //如果机器人位置发生变化,则进行地图合并
        if(abs(ego_position_last.x - ego_position_.x) >= 0.1 || abs(ego_position_last.y - ego_position_.y) >= 0.1 || abs(ego_position_last.z - ego_position_.z) >= 0.1)
        {
            if(initSucessed)
            {
                mapMerge();
                isMoveInited.data = 1;
                move_inited_pub->publish(isMoveInited);
            }
            ego_position_last = ego_position_;
        }

        if(initSucessed)
        {
            globalMap_pub->publish(globalMapData);          
        }
        robot_position_pub->publish(ego_position_);
        rclcpp::spin_some(node_handle);
        rate.sleep(); 
    }
    return 0;
}
