/**
 *  Created by Qingchen Bi on 2022/11/05
 */

#include "../include/plane.h"

// 宏定义:用于连续坐标和离散坐标之间的转换
// 将连续坐标转换为离散网格索引
#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
// 将离散网格索引转换为连续坐标(网格中心点)
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

// x方向偏移
int dx_[4] = {-1, 0, 1, 0};
// y方向偏移
int dy_[4] = {0, 1, 0, -1};
double PointCloudTime = 0;

Eigen::Vector3d center = Eigen::Vector3d::Zero(); 

namespace FitPlane
{
    PlaneMap::PlaneMap(World* world, const float resolution)
    {
        world_ = world;
        // 保存分辨率
        resolution_ = resolution;
        // 初始化
        init();
    }

    // 点云地图回调函数,处理新接收的点云数据
    void PlaneMap::PointCloudMapCallback(const sensor_msgs::msg::PointCloud2& PointCloud_Map)
    {
        // Slam-sim-out or Fast-Lio
        PointCloudTime = rclcpp::Time(PointCloud_Map.header.stamp).seconds();
        // 将ROS点云消息转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(PointCloud_Map, cloud);
        world_->initGridMap(cloud); 

       // 遍历近距离点云,设置障碍物
        for (const auto& pt : world_->cloud_near_)
        {
            Eigen::Vector3d obstacle(pt.x, pt.y, pt.z);
            world_->setObs(obstacle);
        }
        //visualization::visWorld(world_, &world_->Grid_Map_pub); 
       // 初始化平面地图
        InitPlaneMap();
        
       // 计时开始
        timeval start;
        gettimeofday(&start, NULL);

        // 获取平面地图并发布
        getPlaneMap();
       // visSurf(*this, world_->Plane_Map_pub);
        pubPlaneGridMap(*this);
       // 计时结束,计算耗时
        timeval end;
        gettimeofday(&end, NULL);
        double ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
        // fprintf(file, "%lf \n", ms);
    }

    // 初始化函数,设置ROS2发布者和订阅者
    bool PlaneMap::init()
    {
        world_->nh_->declare_parameter<std::string>("Plane_Map_topic", world_->Plane_Map_topic);
        world_->nh_->get_parameter("Plane_Map_topic", world_->Plane_Map_topic);
        world_->PointCloud_Map_sub = world_->nh_->create_subscription<sensor_msgs::msg::PointCloud2>(world_->PointCloud_Map_topic, 10,std::bind(&FitPlane::PlaneMap::PointCloudMapCallback, this, std::placeholders::_1));


        world_->Plane_Map_pub = world_->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(world_->Plane_Map_topic, 1);
        plane_OccMap_pub_ = world_->nh_->create_publisher<nav_msgs::msg::OccupancyGrid>("plane_OccMap", 1);
        trav_cloud_pub = world_->nh_->create_publisher<sensor_msgs::msg::PointCloud2>("local_traversibility_ponit_cloud", 1);
        pubExploredArea = world_->nh_->create_publisher<sensor_msgs::msg::PointCloud2> ("/explored_areas", 5);
        // 设置探索区域的体素滤波器参数
        exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);

        return true;        
    }

    PlaneMap::~PlaneMap()
    {
        clearPlaneMap();
    }
    // 清理平面地图内存
    void PlaneMap::clearPlaneMap()
    {
        if(this->has_PlaneMap_)
        {
            for(int i=0;i < index_num_(0);i++)
            {
                if(Plane_Map_[i] != NULL)
                {
                    delete[] Plane_Map_[i]; 
                    Plane_Map_[i]=NULL; 
                }
            }
            if(Plane_Map_ !=NULL)
            {
                delete[] Plane_Map_;
                Plane_Map_=NULL;
            }            
        }

    }
  // 初始化平面地图
    bool PlaneMap::InitPlaneMap()
    {
        if(this->has_PlaneMap_)
            clearPlaneMap();

        if(world_ != NULL)
        {
            // 获取地图边界
            leftdownbound_ = world_->getLowerBound();
            rightupbound_ = world_->getUpperBound();
            // 计算网格数量
            index_num_ = ((rightupbound_ - leftdownbound_) / resolution_).cast<int>() + Eigen::Vector3i::Ones(); 
             // 分配二维数组内存
            Plane_Map_ = new Plane*[index_num_(0)];
            for(int i = 0; i < index_num_(0); i++)
            {
                Plane_Map_[i] = new Plane[index_num_(1)];
            }
            this->has_PlaneMap_ = true; 
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("Plane"), "No world!");
            this->has_PlaneMap_ = false;
        }

        return this->has_PlaneMap_;
    }
   // 获取平面地图
    bool PlaneMap::getPlaneMap()
    {
        // 计算机器人当前位置对应的网格索引
        int r_x = CONTXY2DISC(world_->ego_position_.x - leftdownbound_(0), resolution_); 
        int r_y = CONTXY2DISC(world_->ego_position_.y - leftdownbound_(1), resolution_);

        // 计算需要处理的区域范围(以机器人为中心的40m×40m区域)
        int lowerx = std::max(0, r_x - int(20.0 / resolution_));
        int lowery = std::max(0, r_y - int(20.0 / resolution_));
        int upperx = std::min(index_num_(0), r_x + int(20.0 / resolution_));
        int uppery = std::min(index_num_(1), r_y + int(20.0 / resolution_));

        // 遍历区域内的每个网格
        for(int i = lowerx; i < upperx; i++)
        {
            for(int j = lowery; j < uppery; j++)
            {
                // 获取网格中心点的世界坐标
                Eigen::Vector3d tmp_P(PlaneMap::index2coord(Eigen::Vector3i(i,j,0)));
                Eigen::Vector3d p_sur;
                // 将点投影到表面,并拟合平面
                if(world_->project2surface(tmp_P(0), tmp_P(1), &p_sur))
                {
                    Plane_Map_[i][j] = FitPlane(p_sur, world_, 0.5/2.0);
                }
            }
        }
        return true;
    }

    // 拟合平面函数
    Plane PlaneMap::FitPlane(Eigen::Vector3d& p_surface,World* world,const double &radius)
    {
        Plane tmp_Plane;
        tmp_Plane.init_coord = Eigen::Vector3d(p_surface(0), p_surface(1), p_surface(2));        

        // 获取球心坐标和分辨率
        Eigen::Vector3d ball_center = world->coordRounding(p_surface);
        float resolution = world->getResolution();
        int fit_num=static_cast<int>(radius/resolution);
         // 创建访问标记数组
        Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> vac(2*fit_num+1,2*fit_num+1);
        int vac_cout_init=(2*fit_num+1)*(2*fit_num+1);
         // 计算点云中心
        // Eigen::Vector3d center= Eigen::Vector3d::Zero(); 
        // 在球形区域内收集点云
        for(int i = -fit_num;i <= fit_num;i++)
        {
            for(int j = -fit_num;j <= fit_num;j++)
            {
                vac(i+fit_num,j+fit_num)=false;
                // 在垂直方向上采样20个点
                for(int k = -10;k <= 10;k++)
                { 
                    Eigen::Vector3d point=ball_center+resolution*Eigen::Vector3d(i,j,k); 
                    if(k==10)
                    {
                        center=point;
                    }
                    // 如果点在边界内且不是空闲空间,则加入平面点集
                    if(world->isInsideBorder(point) && !world->isFree(point))
                    {
                        tmp_Plane.plane_pts.push_back(point);
                        if(!vac(i+fit_num,j+fit_num))
                        {
                            vac(i+fit_num,j+fit_num)=true;
                            vac_cout_init--;
                        }
                    }
                }
            }
        }
         // 获取点云数量
        size_t pt_num=tmp_Plane.plane_pts.size();

        // 如果点数太少,直接返回
        if(pt_num < 10)
            return tmp_Plane;

        for(const auto&pt:tmp_Plane.plane_pts) center += pt; 
        center /= pt_num; 
         // 构建矩阵A用于SVD分解
        Eigen::MatrixXd A(pt_num,3);
        for(size_t i = 0; i < pt_num; i++) A.row(i)=tmp_Plane.plane_pts[i]-center; 
        // 使用SVD分解求解平面法向量
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,Eigen::ComputeFullV);
        tmp_Plane.normal_vector=svd.matrixV().col(2); 
        // 计算平面倾角(与垂直方向的夹角)
        double angle_ = getAngle(tmp_Plane.normal_vector);
        angle_ = angle_ * 180.0 / PI;

        // 计算平面方程参数
        double D = tmp_Plane.normal_vector(0) * center(0) + tmp_Plane.normal_vector(1) * center(1) + tmp_Plane.normal_vector(2) * center(2);
        double H = D - (tmp_Plane.normal_vector(0) * p_surface(0) + tmp_Plane.normal_vector(1) * p_surface(1));
        H = H / tmp_Plane.normal_vector(2); 

        // 计算平面平整度
        float flatness = 0.0;
        double dval = 0.0;
        double sumd = 0.0;
        for(size_t i = 0; i < pt_num; i++)
        {
           dval = tmp_Plane.normal_vector.dot(A.row(i));
           sumd = sumd + dval * dval;
        }
        flatness = sumd / pt_num;   

        // 根据倾角和平整度计算可通行性
        if(angle_ >= max_angle_ || flatness >= max_flatness_)   
            tmp_Plane.traversability = 1; 
        else 
            tmp_Plane.traversability = w1_ * (angle_ / max_angle_) + (1 - w1_) * (flatness / max_flatness_);
        // 保存平面高度和角度
        tmp_Plane.plane_height = H;
        tmp_Plane.plane_angle = angle_;

        return tmp_Plane;
    }
    // 可视化平面点云
    void PlaneMap::visSurf(const PlaneMap &planemap,rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_vis_pub)
    {
        if (surf_vis_pub == NULL)
            return;
        // 创建点云对象
        pcl::PointCloud<pcl::PointXYZRGB> surf_point;
        pcl::PointXYZRGB pt;

        // 遍历所有平面
        for(int i = 0; i < planemap.index_num_[0]; i++)
        {
            for(int j = 0; j < planemap.index_num_[1]; j++)
            {
                // 只处理可通行性不为100的平面
                if(planemap.Plane_Map_[i][j].traversability != 100)
                {
                     // 添加平面上的所有点到点云中
                    for(const auto& point : planemap.Plane_Map_[i][j].plane_pts)
                    {
                        pt.x = point(0);
                        pt.y = point(1);
                        pt.z = point(2);
                        pt.r = pt.g = pt.b = 0;
                        pt.a = 1.0f;

                        surf_point.push_back(pt);        
                    }
                }
            }
        }

        // 设置点云参数
        surf_point.width = surf_point.points.size();
        surf_point.height = 1;
        surf_point.is_dense = true;
        // 转换为ROS消息并发布

        sensor_msgs::msg::PointCloud2 map_vis;
        pcl::toROSMsg(surf_point, map_vis);

        map_vis.header.frame_id = "map";
        surf_vis_pub->publish(map_vis);
    }

    // 发布平面栅格地图
    bool PlaneMap::pubPlaneGridMap(const PlaneMap &planemap)
    {
         // 设置栅格地图参数
        plane_Occmap_.info.height = planemap.index_num_(1) * int(resolution_ / resolution_); // 0.5
        plane_Occmap_.info.width = planemap.index_num_(0) * int(resolution_ / resolution_);// 0.5
        plane_Occmap_.info.resolution = resolution_;// 0.5
        plane_Occmap_.header.frame_id = "map";
        plane_Occmap_.info.origin.position.x = leftdownbound_(0);
        plane_Occmap_.info.origin.position.y = leftdownbound_(1);
        // 初始化地图数据
        plane_Occmap_.data.clear();
        std::vector<int8_t> tmp(plane_Occmap_.info.height * plane_Occmap_.info.width, -1);
        plane_Occmap_.data = tmp;

        // 遍历每个栅格
        for(unsigned int i = 0; i < plane_Occmap_.info.width; i++)
        {
            for(unsigned int j = 0; j < plane_Occmap_.info.height; j++)
            {   
                // 计算栅格中心点的世界坐标
                double x = DISCXY2CONT(i, resolution_) + leftdownbound_(0);// 0.5
                double y = DISCXY2CONT(j, resolution_) + leftdownbound_(1);// 0.5

                // 计算对应的平面地图索引
                int l_x = CONTXY2DISC(x - leftdownbound_(0), resolution_); 
                int l_y = CONTXY2DISC(y - leftdownbound_(1), resolution_);

                 // 如果索引在有效范围内
                if(l_x >= 0 && l_x < planemap.index_num_(0) && l_y >= 0 && l_y < planemap.index_num_(1))
                {
                    // 根据可通行性设置栅格值
                    if(planemap.Plane_Map_[l_x][l_y].traversability != 100)
                    {
                        if(planemap.Plane_Map_[l_x][l_y].traversability == 1)// 0.9 && planemap.Plane_Map_[i][j].traversability <= 1.0)
                        {
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j]= occThre_;
                        }
                        else if(planemap.Plane_Map_[l_x][l_y].traversability == 10)// 0.9 && planemap.Plane_Map_[i][j].traversability <= 1.0)
                        {
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j]= nobs_;
                        }
                        else
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j] = 100 * planemap.Plane_Map_[l_x][l_y].traversability;   

                    }
                }
            }
        }

         // 保存原始占用栅格地图
        Original_plane_Occmap_ = plane_Occmap_;
        // 定义8个方向的偏移量
        int d_x8[8] = {-1, 0, 0, 1, 1, -1, 1, -1};
        int d_y8[8] = {0, 1, -1, 0, 1, -1, -1, 1};
        // 获取地图尺寸
        width_ = plane_Occmap_.info.width;
        height_ = plane_Occmap_.info.height;
         // 用于平滑处理的变量
        int sum = 0;
        int count = 0;

         // 第一次遍历:平滑处理障碍物周围的栅格
        for(int i = 0; i < width_; i++)
        {
            for(int j = 0; j < height_; j++)
            {
                if(Original_plane_Occmap_.data[i + j * width_] == occThre_) 
                {
                     // 检查8个邻域
                    for(int d = 0; d < 8; d++)
                    {
                        int x_tmp = i + d_x8[d];
                        int y_tmp = j + d_y8[d];
                        if(isInBorder(x_tmp, y_tmp))
                        {
                            // 计算邻域栅格的平均值
                            if(Original_plane_Occmap_.data[x_tmp + y_tmp * width_] == -1)
                            {
                                sum = sum + 0;
                                count++;
                            }
                            else if(Original_plane_Occmap_.data[x_tmp + y_tmp * width_] != occThre_)
                            {
                                sum = Original_plane_Occmap_.data[x_tmp + y_tmp * width_] + sum;
                                count++;
                            }
                        }
                    }
                    // 如果所有8个邻域都有效,更新中心栅格的值
                    if(count == 8)
                    {
                        plane_Occmap_.data[i + j * width_] = (plane_Occmap_.data[i + j * width_] + sum ) / (count + 1);
                    }
                    sum = 0;    
                    count = 0;                    
                }
            }
        }
        //第二次遍历:处理障碍物和不可通行区域周围的栅格
        for(int i = 0; i < width_; i++)
        {
            for(int j = 0; j < height_; j++)
            {
                if(plane_Occmap_.data[i + j * width_] == occThre_ || plane_Occmap_.data[i + j * width_] == nobs_) // 
                {
                     // 检查8个邻域
                    for(int d = 0; d < 8; d++)
                    {
                        int x_tmp = i + d_x8[d];
                        int y_tmp = j + d_y8[d];
                        if(isInBorder(x_tmp, y_tmp))
                        {
                             // 更新邻域栅格的可通行性
                            if(plane_Occmap_.data[x_tmp + y_tmp * width_] < flatThre_)
                            {
                                if(plane_Occmap_.data[i + j * width_] == occThre_)
                                    plane_Occmap_.data[x_tmp + y_tmp * width_] = flatThre_;
                                else
                                {
                                    plane_Occmap_.data[x_tmp + y_tmp * width_] =  nobs_ - 1;
                                }
                            }
                        }
                    }
                } 
            }
        }
         // 更新地图时间戳并发布
        plane_Occmap_.header.stamp =rclcpp::Time(static_cast<uint64_t>(PointCloudTime * 1e9));
        plane_OccMap_pub_->publish(plane_Occmap_);

        // 计算机器人当前位置的网格索引
        int r_x = CONTXY2DISC(world_->ego_position_.x - leftdownbound_(0), resolution_); 
        int r_y = CONTXY2DISC(world_->ego_position_.y - leftdownbound_(1), resolution_);
       // 创建用于可视化的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr trav_point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
       // 遍历机器人周围25x25的区域
        for(int i = r_x - 12; i < r_x + 13; i++)
        {
            for(int j = r_y - 12; j < r_y + 13; j++)
            {
                if(isInBorder(i, j))
                {
                    // 处理不可通行区域
                    if(plane_Occmap_.data[i + j * width_] >= 99)
                    {
                        // 计算世界坐标
                        double x = DISCXY2CONT(i, resolution_) + leftdownbound_(0);// 0.5
                        double y = DISCXY2CONT(j, resolution_) + leftdownbound_(1);// 0.5

                        // 添加高亮点(不可通行区域)
                        pcl::PointXYZI reg_point;
                        reg_point.x = x;
                        reg_point.y = y;
                        reg_point.z = world_->ego_position_.z + 0.1 * 6;
                        reg_point.intensity = 1;
                        trav_point_cloud->points.push_back(reg_point);
                        
                    }
                   // 处理可通行区域
                    else
                    {
                        // 计算世界坐标
                        double x = DISCXY2CONT(i, resolution_) + leftdownbound_(0);// 0.5
                        double y = DISCXY2CONT(j, resolution_) + leftdownbound_(1);// 0.5

                    // 添加低亮点(可通行区域)
                        pcl::PointXYZI reg_point;
                        reg_point.x = x;
                        reg_point.y = y;
                        reg_point.z = world_->ego_position_.z - 0.1;
                        reg_point.intensity = 0.2;
                        trav_point_cloud->points.push_back(reg_point);
                    }
                }
            }
        }
         // 将点云转换为ROS消息并发布
        sensor_msgs::msg::PointCloud2 trav_point_cloud_msg;
        pcl::toROSMsg(*trav_point_cloud, trav_point_cloud_msg);
        trav_point_cloud_msg.header.stamp= rclcpp::Time(0);
        trav_point_cloud_msg.header.frame_id = "map";
        trav_cloud_pub->publish(trav_point_cloud_msg);// publish downsample point cloud

        return true;
    }

 // 计算向量与垂直方向的夹角
    double PlaneMap::getAngle(Eigen::Vector3d &plane_vector)
    {
        // 垂直向上的单位向量
        Eigen::Vector3d n1(0,0,1);
	   // 计算两个向量的夹角的余弦值
         double cos_ = abs(n1(0) * plane_vector(0) + n1(1) * plane_vector(1) + n1(2) * plane_vector(2)) / 
			    (sqrt(n1(0) * n1(0) + n1(1) * n1(1) + n1(2) * n1(2)) * 
         		sqrt(plane_vector(0) * plane_vector(0) + plane_vector(1) * 
	    		plane_vector(1) + plane_vector(2) * plane_vector(2)));
                 // 返回弧度制的角度
        double angle = std::acos(cos_);
	return angle;
    }

}
