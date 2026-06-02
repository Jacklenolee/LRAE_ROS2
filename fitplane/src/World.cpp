/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 *  Author: jianzhuozhu
 *  Date: 2021-7-24
 * 
 *  Modified by Qingchen Bi on 2022/11/05
 */

#include "../include/plane.h"

using namespace std;
using namespace Eigen;

namespace FitPlane
{

World::World(const float &resolution,const rclcpp::Node::SharedPtr nh)
{
    // 初始化边界为无穷大
    lowerbound_=INF*Vector3d::Ones(); 
    upperbound_=-INF*Vector3d::Ones();
    // 初始化索引计数为0
    idx_count_=Vector3i::Zero(); 
    resolution_ = resolution;
    nh_ = nh;
    // 创建TF缓存和监听器
    buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    nh_->declare_parameter<std::string>("PointCloud_Map_topic", PointCloud_Map_topic);//定义点云地图的话题名称，
    nh_->declare_parameter<std::string>("Grid_Map_topic", Grid_Map_topic);//定义栅格地图的话题名称
    nh_->declare_parameter<std::string>("PointCloud_topic", PointCloud_topic);//定义原始点云的话题名称
    nh_->declare_parameter<bool>("use_ex_range", use_ex_range_);//定义是否使用探索范围限制
    nh_->declare_parameter<double>("ex_robot_back", ex_robot_back_);//定义机器人在x轴负方向上的最远探索距离
    nh_->declare_parameter<double>("ex_robot_front", ex_robot_front_);//定义机器人在x轴正方向上的最远探索距离
    nh_->declare_parameter<double>("ex_robot_right", ex_robot_right_);//定义机器人在y轴正方向上的最远探索距离
    nh_->declare_parameter<double>("ex_robot_left", ex_robot_left_);//定义机器人在y轴负方向上的最远探索距离

    nh_->get_parameter("PointCloud_Map_topic", PointCloud_Map_topic);
    nh_->get_parameter("Grid_Map_topic", Grid_Map_topic);
    nh_->get_parameter("PointCloud_topic", PointCloud_topic);
    nh_->get_parameter("use_ex_range", use_ex_range_);
    nh_->get_parameter("ex_robot_back", ex_robot_back_);
    nh_->get_parameter("ex_robot_front", ex_robot_front_);
    nh_->get_parameter("ex_robot_right", ex_robot_right_);
    nh_->get_parameter("ex_robot_left", ex_robot_left_);

    // 创建网格地图发布器
    Grid_Map_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(Grid_Map_topic, 1);
}
// 析构函数,清理地图内存
World::~World()
{
    clearMap();
}
// 清理地图内存的方法
void World::clearMap()
{
    if(has_map_)
    {
        for(int i=0;i < idx_count_(0);i++)
        {
            for(int j=0;j < idx_count_(1);j++)
            {
                delete[] grid_map_[i][j]; 
                grid_map_[i][j]=NULL; 
            }
            delete[] grid_map_[i];
            grid_map_[i]=NULL;
        }
        delete[] grid_map_;
        grid_map_=NULL;
    }
}

// 使用给定边界初始化网格地图
void World::initGridMap(const Vector3d &lowerbound,const Vector3d &upperbound)
{
    lowerbound_=lowerbound;
    upperbound_=upperbound;
     // 计算网格数量
    idx_count_=((upperbound_-lowerbound_)/resolution_).cast<int>()+Eigen::Vector3i::Ones(); 
     // 分配三维布尔数组内存
    grid_map_=new bool**[idx_count_(0)];
    for(int i=0;i < idx_count_(0);i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j=0;j < idx_count_(1);j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            // 初始化所有网格为true(空闲)
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool)); 
        }
    }
    has_map_=true; 
}

// 使用点云数据初始化网格地图
void World::initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{   
    if(cloud.points.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("World"),"Can not initialize the map with an empty point cloud!");
        return;
    }
    // 清理现有地图
    clearMap();
    cloud_near_.clear();
     // 获取机器人当前位置
    GetRobotPosition();
    // 遍历点云中的每个点
    for(const auto&pt:cloud.points)
    {
        // 如果启用了探索范围限制,检查点是否在范围内
        if(use_ex_range_)
        {
            if(pt.x < ex_robot_back_ || pt.x > ex_robot_front_ || pt.y < ex_robot_right_ || pt.y > ex_robot_left_)
                continue;            
        }
        // 检查点是否在最小范围内
        if(abs(pt.x - ego_position_.x) > minrange_ || abs(pt.y - ego_position_.y) > minrange_)
            continue;
        else
        {
            // 将符合条件的点加入近距离点云
            cloud_near_.points.push_back(pt);        
        }
    }
    // 初始化边界值
    float minx = 10000;
    float miny = 10000;
    float minz = 10000; 
    float maxx = -10000;
    float maxy = -10000;
    float maxz = -10000;   
    // 遍历近距离点云,更新边界值 
    for(const auto&pt:cloud_near_.points)
    {
        // 更新最小边界值
        if(pt.x < minx)
        {
            minx = pt.x;
            lowerbound_(0)=minx;
        } 
        if(pt.y < miny)
        {
            miny = pt.y;
            lowerbound_(1)=miny;
        } 
        if(pt.z < minz)
        {
            minz = pt.z;
            lowerbound_(2)=minz;
        } 

        if(pt.x > maxx)
        {
            maxx = pt.x;
            upperbound_(0)=maxx;
        } 
        if(pt.y > maxy)
        {
            maxy = pt.y;
            upperbound_(1)=maxy;
        } 
         // 更新z轴最大值(加1米余量)
        if(pt.z + 1.0 > maxz)
        {
            maxz = pt.z + 1;
            upperbound_(2)=maxz;
        } 
    }
  // 计算网格数量
    idx_count_ = ((upperbound_-lowerbound_)/resolution_).cast<int>() + Eigen::Vector3i::Ones();
  // 分配三维网格地图内存
    grid_map_=new bool**[idx_count_(0)];
    for(int i = 0 ; i < idx_count_(0) ; i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j = 0 ; j < idx_count_(1) ; j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            // 初始化所有网格为true(空闲)
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
        }
    }
    has_map_=true;
}
// 在指定位置设置障碍物
void World::setObs(const Vector3d &point)
{   
    // 将坐标转换为网格索引
    Vector3i idx=coord2index(point);
     // 将该位置标记为被占用(false)
    grid_map_[idx(0)][idx(1)][idx(2)]=false; 
}
// 检查指定位置是否空闲
bool World::isFree(const Vector3d &point)
{
   // 将坐标转换为网格索引
    Vector3i idx = coord2index(point);
    // 检查位置是否在边界内且为空闲
    bool is_free = isInsideBorder(idx) && grid_map_[idx(0)][idx(1)][idx(2)];
    return is_free;
}
// 将坐标四舍五入到网格中心
Vector3d World::coordRounding(const Vector3d & coord)
{
    return index2coord(coord2index(coord));
}

// 在给定x,y位置寻找表面高度
bool World::project2surface(const float &x,const float &y,Vector3d* p_surface)
{
    bool ifsuccess=false;
    // 检查x,y是否在地图边界内
    if(x>=lowerbound_(0) && x<=upperbound_(0) && y>=lowerbound_(1) && y<=upperbound_(1))
    {
        // 从最低高度开始向上搜索
        for(float z = lowerbound_(2) ; z < upperbound_(2) ; z+=resolution_)
        {
            // 找到第一个被占用格子上方的空闲格子
            if( !isFree(x,y,z) && isFree(x,y,z+resolution_) )
            {
                *p_surface=Vector3d(x,y,z); 
                ifsuccess=true;
                break;
            }
        }
    }
    return ifsuccess;
}
// 检查索引是否在地图边界内
bool World::isInsideBorder(const Vector3i &index)
{
    return index(0) >= 0 &&
           index(1) >= 0 &&
           index(2) >= 0 && 
           index(0) < idx_count_(0)&&
           index(1) < idx_count_(1)&&
           index(2) < idx_count_(2);
}
}

