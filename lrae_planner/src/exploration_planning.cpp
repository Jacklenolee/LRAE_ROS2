#include "exploration_planning.h"
#include <cstring>
#include "time.h"
#include <Eigen/Core>
#include "nav_msgs/msg/path.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int d_x4[4] = {-1, 0, 0, 1};
int d_y4[4] = {0, 1, -1, 0};
int hsd_x6[6] = {-1, 0, 1, -1, 0 ,1};
int hsd_y6[6] = {1, 1, 1, -1, -1, -1};
int wsd_x6[6] = {-1, -1, -1, 1, 1, 1};
int wsd_y6[6] = {1, 0, -1, 1, 0, -1};
int d_x8[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
int d_y8[8] = {1, 1, 1, 0, -1, -1, -1, 0};
int d_x16[16] = {-2,-2,-2,-2,-2, -1,-1, 0, 0, 1, 1, 2,2,2,2,2};
int d_y16[16] = {-2,-1,0, 1,  2, -2, 2, -2,2, 2,-2, -2,-1,0,2,1};

double run_time_s = 0.0;

namespace lrae_planner_ns
{
ExplorationPlanning::ExplorationPlanning(rclcpp::Node::SharedPtr nh):
has_robot_position_(false),
has_map_(false),
has_tra_map_(false)
{
    nh_ = nh;
    ns_ = nh_->get_name();
    // 创建路径规划器对象
    cal_cost_planner_ = new GraphSearch();
    buffer_ = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    initialize();
}

ExplorationPlanning::~ExplorationPlanning()
{
    if(cal_cost_planner_)
    {
        delete cal_cost_planner_;
    }
}

bool ExplorationPlanning::initialize()
{
    //边界索引初始化
    min_left_index_.x = 10000;
    min_left_index_.y = 10000;
    max_right_index_.x = -10000;
    max_right_index_.y = -10000;

    nh_->declare_parameter<double>("angle_pen", angle_pen_);
    //声明角度惩罚参数
    nh_->declare_parameter<int>("update_cen_thre", update_cen_thre_);
    //声明更新中心阈值参数
    nh_->declare_parameter<int>("unknown_num_thre", unknown_num_thre_);
    //声明未知区域阈值参数
    nh_->declare_parameter<double>("minrange", minrange_);
    //声明最小范围参数
    nh_->declare_parameter<bool>("limit_max_square", limit_max_square_);
    //声明最大方形限制参数
    nh_->declare_parameter<bool>("use_go_end_nearest", use_go_end_nearest_);
    //声明使用终点最近参数    
    nh_->declare_parameter<double>("end_neacen_disthre", end_neacen_disthre_);
    //声明终点最近距离阈值参数
    nh_->declare_parameter<double>("end_cur_disrate", end_cur_disrate_);
    //声明终点当前距离率参数

    nh_->get_parameter("angle_pen", angle_pen_);
    nh_->get_parameter("update_cen_thre", update_cen_thre_);
    nh_->get_parameter("unknown_num_thre", unknown_num_thre_);
    nh_->get_parameter("minrange", minrange_);
    nh_->get_parameter("limit_max_square", limit_max_square_);
    nh_->get_parameter("use_go_end_nearest", use_go_end_nearest_);
    nh_->get_parameter("end_neacen_disthre", end_neacen_disthre_);
    nh_->get_parameter("end_cur_disrate", end_cur_disrate_);

    update_cen_count_ = update_cen_thre_ - 1; 
    //当update_cen_count_达到update_cen_thre_设定的阈值时，会触发一次中心点的更新
    // 创建一个周期为500毫秒的定时器，并绑定执行函数
    execution_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ExplorationPlanning::execute, this)); 
    
    // 创建一个订阅者，订阅/plane_OccMap话题，并绑定回调函数traversibilityMapCallback
    traversibility_map_sub_ = nh_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversibility_map_topic_, 100,
    std::bind(&ExplorationPlanning::traversibilityMapCallback,this, std::placeholders::_1));
   
    // 创建一个订阅者，订阅/globalMap话题，并绑定回调函数globalMapCallback
    global_map_sub_ = nh_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_map_topic_, 100,
    std::bind(&ExplorationPlanning::globalMapCallback,this, std::placeholders::_1));

    exploration_path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("exporation_path", 100);
    exploration_route_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("RoutePath", 100);
    //机器人位置标记(红色点)，Rviz中显示
    robot_position_marker_ =
        std::make_shared<visual::Marker>(nh_, "pointvisiual", "map");
    robot_position_marker_->SetAction(visualization_msgs::msg::Marker::ADD);
    robot_position_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
    robot_position_marker_->SetScale(0.1, 0.1, 0.1);
    robot_position_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);

    // 中心点标记(绿色点)，Rviz中显示
    centroids_marker_ =
        std::make_shared<visual::Marker>(nh_, "centroids", "map");
    centroids_marker_->SetAction(visualization_msgs::msg::Marker::ADD);
    centroids_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
    centroids_marker_->SetScale(0.2, 0.2, 0.2);
    centroids_marker_->SetColorRGBA(0.0, 1.0, 0.0, 1.0);

    // 窗口标记(蓝色线)，Rviz中显示
    windows_marker_ =
        std::make_shared<visual::Marker>(nh_, "windows", "map");
    windows_marker_->SetAction(visualization_msgs::msg::Marker::ADD);
    windows_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
    windows_marker_->SetScale(0.1, 0.1, 0.1);
    windows_marker_->SetColorRGBA(0.0, 0.0, 1.0, 1.0);

    // 视点标记(青色点)，Rviz中显示
    viewpoints_marker_ =
            std::make_shared<visual::Marker>(nh_, "viewpoints", "map");
    viewpoints_marker_->SetAction(visualization_msgs::msg::Marker::ADD);
    viewpoints_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
    viewpoints_marker_->SetScale(0.2, 0.2, 0.2);
    viewpoints_marker_->SetColorRGBA(0.0, 1.0, 1.0, 1.0);

    // 选择中心点标记(紫色点)，Rviz中显示
    choose_centroids_marker_ =
            std::make_shared<visual::Marker>(nh_, "choosecentroids", "map");
    choose_centroids_marker_->SetAction(visualization_msgs::msg::Marker::ADD);
    choose_centroids_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
    choose_centroids_marker_->SetScale(0.4, 0.4, 0.2);
    choose_centroids_marker_->SetColorRGBA(1.0, 0.0, 1.0, 1.0);

    // 选择视点标记(黄色点)，Rviz中显示
    choose_viewpoints_marker_ =
            std::make_shared<visual::Marker>(nh_, "chooseviewpoints", "map");
    choose_viewpoints_marker_->SetAction(visualization_msgs::msg::Marker::ADD);
    choose_viewpoints_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
    choose_viewpoints_marker_->SetScale(0.3, 0.3, 0.2);
    choose_viewpoints_marker_->SetColorRGBA(1.0, 1.0, 0.0, 1.0);

    return true;
}

void ExplorationPlanning::getRobotPosition()
{
    geometry_msgs::msg::Pose current_pose_ros;
    geometry_msgs::msg::TransformStamped transform_pose;
    tf2::Quaternion quat; 
    double roll,pitch,yaw;
    geometry_msgs::msg::Point point_temp_vis;
    try
    {
        transform_=buffer_->lookupTransform("map", "base_link",tf2::TimePointZero);
        ego_position_.x = transform_.transform.translation.x;
        ego_position_.y = transform_.transform.translation.y;
        point_temp_vis.x = ego_position_.x;
        point_temp_vis.y = ego_position_.y;
        tf2::fromMsg(transform_.transform.rotation,quat); 
        tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw); 
        ego_position_.z = yaw;
        has_robot_position_ = true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_planning"), "Caught exception: %s", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    if(has_robot_position_ )
    {
        robot_position_marker_->marker_.points.push_back(point_temp_vis);
        robot_position_marker_->Publish();
    }
}

std::vector<int> ExplorationPlanning::getRouteOrder()
{
    double lambda = 3.0;
    std::vector<int> ids;
    for (int i = 0; i <= centroids_.size(); ++i) {
        ids.push_back(i);
    }
    std::vector<int> init_route = ids;

    std::vector<double> gains;
    gains.push_back(0.0);
    for(int i = 0; i < centroids_.size(); i++)
    {
        gains.push_back((double)(centroids_[i].unknownnum));
    }

    std::vector<std::vector<double>> cost_matrix = std::vector<std::vector<double >>(centroids_.size() + 1, std::vector<double>(centroids_.size() + 1, 100000.0));
    for(int i = 0; i < centroids_.size() + 1; i++)
    {
        for(int j = 0; j < centroids_.size() + 1; j++)
        {
            if(j == 0)
                cost_matrix[i][j] = 0.0;
            else if(i == j)
                cost_matrix[i][j] = 0.0;
            else if(cost_matrix[j][i] != 100000.0)
                cost_matrix[i][j] = cost_matrix[j][i];    
            else
            {
                if(i == 0) // Robot
                {
                    double angleRAndC = utils_ns::getAngleRobot(centroids_[j - 1].centroidposition, ego_position_);
                    double angle_pen_item = ((1 - angle_pen_) * (log(angleRAndC / M_PI + 1) / log(2)) + angle_pen_);
                   
                    utils_ns::Index2 startR, endC;
                    startR.x = CONTXY2DISC(ego_position_.x - global_map_.info.origin.position.x, global_map_.info.resolution);
                    startR.y = CONTXY2DISC(ego_position_.y - global_map_.info.origin.position.y, global_map_.info.resolution);
                    endC.x = CONTXY2DISC(centroids_[j - 1].centroidposition.x - global_map_.info.origin.position.x, global_map_.info.resolution);
                    endC.y = CONTXY2DISC(centroids_[j - 1].centroidposition.y - global_map_.info.origin.position.y, global_map_.info.resolution);
                    int pathcostRC;
                    std::vector<utils_ns::PointInt> tmppath;
                    cal_cost_planner_->SearchforCost(startR.x, startR.y, endC.x, endC.y, global_map_, 
                                                    global_map_.info.width, global_map_.info.height, 98, pathcostRC, tmppath);
                    double pathdisRC = (double)(pathcostRC) / 10.0 * global_map_.info.resolution;
                    cost_matrix[i][j] = angle_pen_item * pathdisRC; 
                }
                else
                {
                    cost_matrix[i][j] = sqrt(DISTANCE2(centroids_[i - 1].centroidposition.x, centroids_[i - 1].centroidposition.y, 
                                                        centroids_[j - 1].centroidposition.x, centroids_[j - 1].centroidposition.y));
                }
            }
        }
    }

    Two_Opt two_opt_solve(init_route, gains, cost_matrix, lambda);
    two_opt_solve.solve();
    auto best_route = two_opt_solve.best_route_;
    // ROS_INFO("the best route is:");
    // for (auto &i:best_route) {
    //     ROS_INFO("%d", i);
    // }
    return best_route;
    // ROS_INFO("the best unity is %.10f", two_opt_solve.best_unity_);
}

void ExplorationPlanning::traversibilityMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    traversibility_map_ = *msg;
    has_tra_map_ = true;  
}

void ExplorationPlanning::globalMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    global_map_ = *msg;
    resolution_ = msg->info.resolution;
    ori_x_ = msg->info.origin.position.x;
    ori_y_ = msg->info.origin.position.y;
    width_ = msg->info.width;
    height_ = msg->info.height;
    has_map_ = true;
}


//过自适应窗口大小来寻找合适的探索区域中心点,并提供可视化支持。
bool ExplorationPlanning::getCentroid(const ExactWindow& ew)
{ 
    std::vector<ExactWindow> ews; // sub_ews
     //存储子窗口
    // get Square
    utils_ns::Index2 robot_index;
    //机器人位置索引
    robot_index.x = CONTXY2DISC(ego_position_.x - ori_x_, resolution_);
    //将机器人X位置转换为栅格索引
    robot_index.y = CONTXY2DISC(ego_position_.y - ori_y_, resolution_);
    //将机器人位置转换为栅格索引
    bool hasfind = false;
    //是否找到合适的窗口大小
    int wun_count = 0;
    // 宽度方向上未知区域计数
    int hun_count = 0;
    // 高度方向上未知区域计数
    int MaxWIndex = 1;
    // 最大宽度索引
    int MaxHIndex = 1;
    // 最大高度索引
    bool wfind = false;
    // 是否找到合适的宽度
    bool hfind = false;
    // 是否找到合适的高度


    // 如果未找到合适的窗口大小且最大宽度索引和最大高度索引小于探索地图的宽度和高度
    //     检查6个方向上的点是否为未知区域(-1)
    // 根据未知区域数量决定是否继续扩大搜索范围，设置hasfind、hfind和wfind标志位
    while(!hasfind && MaxWIndex < traversibility_map_.info.width && MaxHIndex < traversibility_map_.info.height)
    {
        wun_count = 0;
        hun_count = 0;
        for(int i = 0; i < 6; i++)
        {
            utils_ns::Index2 TmpWIndex, TmpHIndex;
            TmpWIndex.x = robot_index.x + wsd_x6[i] * MaxWIndex;
            TmpWIndex.y = robot_index.y + wsd_y6[i] * MaxWIndex;
            TmpHIndex.x = robot_index.x + hsd_x6[i] * MaxHIndex;
            TmpHIndex.y = robot_index.y + hsd_y6[i] * MaxHIndex;
            if(!wfind && isInBorder(TmpWIndex.x, TmpWIndex.y)) 
            {
                if(global_map_.data[TmpWIndex.x + TmpWIndex.y * width_] == -1)
                {
                    wun_count++;
                }
            }
            if(!hfind && isInBorder(TmpHIndex.x, TmpHIndex.y))
            {
                if(global_map_.data[TmpHIndex.x + TmpHIndex.y * width_] == -1)
                {
                    hun_count++;
                }
            }
        }
        if(wun_count >= 2 && hun_count >= 2)
        {
            hfind = true;
            wfind = true;
            hasfind = true;
        }
        else if(wun_count >= 2 && hun_count < 2)
        {
            wfind = true;
            if(!hfind)
                MaxHIndex++;
        }
        else if(wun_count < 2 && hun_count >= 2)
        {
            hfind = true;
            if(!wfind)
                MaxWIndex++;
        }
        else
        {
            if(!wfind)
                MaxWIndex++;          
            if(!hfind)
                MaxHIndex++;
        }
        if(hfind && wfind)    
        {
            hasfind = true;
        }
    }
   
   //如果没找到合适的宽度/高度,设为0，找到后进行边界调整(减4)，确保窗口为正方形
    if(!wfind)
    {
        MaxWIndex = 0;
    }
    else
    {
        MaxWIndex = (MaxWIndex - 4) > 0 ? (MaxWIndex - 4) : 0;
    }
    if(!hfind)
    {
        MaxHIndex = 0;
    }
    else
    {
        MaxHIndex = (MaxHIndex - 4) > 0 ? (MaxHIndex - 4) : 0;
    }

    if(MaxWIndex < MaxHIndex) // TODO At present, it is only a square, not a rectangle
        MaxHIndex = MaxWIndex;
    else
        MaxWIndex = MaxHIndex;

    //限制最大窗口大小
    if(limit_max_square_)
    {
        if(MaxWIndex > (int)(10.0 / global_map_.info.resolution))
        {
            MaxWIndex = (int)(10.0 / global_map_.info.resolution);
            MaxHIndex = (int)(10.0 / global_map_.info.resolution);
        }
    }
    //将大窗口划分为8个子窗口。计算每个子窗口的位置和大小。将子窗口添加到ews向量中
    int w, h;
    // 1
    w = std::max(robot_index.x - ew.oriindx - MaxWIndex, 0);
    h = std::max(robot_index.y - ew.oriindy - MaxHIndex, 0);
    ews.push_back(ExactWindow(ew.oriindx, ew.oriindy, w, h));
    // 2
    ews.push_back(ExactWindow(ew.oriindx + w, ew.oriindy, 2 * MaxWIndex, h));
    // 3
    int w2;
    w2 = std::max(ew.winwithd - w - 2 * MaxWIndex, 0);
    ews.push_back(ExactWindow(ew.oriindx + w + 2 * MaxWIndex, ew.oriindy, w2, h));
    // 4
    ews.push_back(ExactWindow(ew.oriindx, ew.oriindy + h, w, 2 * MaxHIndex));
    // 5
    ews.push_back(ExactWindow(ew.oriindx + w + 2 * MaxWIndex, ew.oriindy + h, w2, 2 * MaxHIndex));
    // 6
    int h2;
    h2 = std::max(ew.winheight - h - 2 * MaxHIndex, 0);
    ews.push_back(ExactWindow(ew.oriindx, ew.oriindy + h + 2 * MaxHIndex, w, h2));
    // 7
    ews.push_back(ExactWindow(ew.oriindx + w, ew.oriindy + h + 2 * MaxHIndex, 2 * MaxWIndex, h2));
    // 8
    ews.push_back(ExactWindow(ew.oriindx + w + 2 * MaxWIndex, ew.oriindy + h + 2 * MaxHIndex, w2, h2));

    //使用windows_marker_绘制窗口边界线。发布标记消息用于显示
    geometry_msgs::msg::Point p1 = index2coord(ew.oriindx + w, ew.oriindy);
    geometry_msgs::msg::Point p2 = index2coord(ew.oriindx + w, ew.oriindy + ew.winheight);
    windows_marker_->PushBackLine(p1, p2);
    p1 = index2coord(ew.oriindx + w + 2 * MaxWIndex, ew.oriindy);
    p2 = index2coord(ew.oriindx + w + 2 * MaxWIndex, ew.oriindy + ew.winheight);
    windows_marker_->PushBackLine(p1, p2);
    p1 = index2coord(ew.oriindx, ew.oriindy + h);
    p2 = index2coord(ew.oriindx + ew.winwithd, ew.oriindy + h);
    windows_marker_->PushBackLine(p1, p2);
    p1 = index2coord(ew.oriindx, ew.oriindy + h + 2 * MaxHIndex);
    p2 = index2coord(ew.oriindx + ew.winwithd, ew.oriindy + h + 2 * MaxHIndex);
    windows_marker_->PushBackLine(p1, p2);   
    windows_marker_->Publish();
    
    centroids_.clear();
    windows_marker_->marker_.points.clear();
    int subwinorder = 1;

    //遍历ews向量中的每个子窗口，对每个子窗口调用getCentroidHelper()，如果失败则尝试将窗口分割成更小的子窗口
    while(!ews.empty())
    {    
        auto current_ew = ews.rbegin();
        int cur_ew_x = current_ew->oriindx;
        int cur_ew_y = current_ew->oriindy;
        int w = current_ew->winwithd;
        int h = current_ew->winheight;
        ews.pop_back();
        if(!getCentroidHelper(cur_ew_x, cur_ew_y, w, h, subwinorder)) 
        { 
            if(w > h)
            {
                if(w / 2  * h >= 9)
                {
                    ews.push_back(ExactWindow(cur_ew_x, cur_ew_y, w / 2, h));   
                }
            }
            else
            {
                if(h / 2 * w >= 9)
                {
                    ews.push_back(ExactWindow(cur_ew_x, cur_ew_y, w, h / 2));   
                }
            }
        }
        else
            subwinorder++;
    }

   //如果找到中心点,清除并更新标记点，发布中心点和视点的可视化标记，返回是否成功找到中心点
    if(!centroids_.empty())
    {
        centroids_marker_->marker_.points.clear();
        viewpoints_marker_->marker_.points.clear();
        for(int i = 0; i < centroids_.size(); i++)
        {       
            centroids_marker_->marker_.points.push_back(centroids_[i].centroidposition);
            if(!centroids_[i].viewpoint_fir.empty())
            {
                for(int j = 0; j < centroids_[i].viewpoint_fir.size(); j++)
                {
                    viewpoints_marker_->marker_.points.push_back(centroids_[i].viewpoint_fir[j].position);
                }
            }
        }
        centroids_marker_->Publish();  
        viewpoints_marker_->Publish();
        return true;
    }
    else
        return false;
}

bool ExplorationPlanning::getCentroidHelper(const int ori_x, const int& ori_y, const int& w, const int& h, const int& winoder)
{
    int count = 0;
    //计数器
    CellPoint cellpoint;
    //用于存储单个栅格点信息
    Centroid tmp_cen;
    //临时存储中心点信息
    utils_ns::ViewPoint tmp_viewpoint;
    //临时存储视点信息
    tmp_cen.centroidposition.x = 0;
    //初始化中心点位置
    tmp_cen.centroidposition.y = 0;
    tmp_cen.viewpoint_fir.clear();
    //清空视点信息
    in_win_Unpoints_.clear();
    //清空视点列表
    //双重循环遍历窗口内的所有栅格
    for(int i = ori_x; i < ori_x + w; i++)
    {            
        for(int j = ori_y; j < ori_y + h; j++)
        {    
            if(isInBorder(i, j))
            //检查栅格是否在窗口内
            {
                if(global_map_.data[i + j * width_] == -1) 
                //检查栅格是否为未知区域
                {
                    //遍历8个方向，检查每个方向的栅格是否为障碍物，如果是则计数器减100，否则计数器加1
                    int count_thre = 0;
                    for(int d = 0; d < 8; d++)
                    {
                        int x_tmp = i + d_x8[d];
                        int y_tmp = j + d_y8[d];
                        if(isInBorder(x_tmp, y_tmp))
                        {
                            if(global_map_.data[x_tmp + y_tmp * width_] >= 90) 
                            {
                                count_thre = -100;
                                break;
                            }
                            else if(global_map_.data[x_tmp + y_tmp * width_] != -1 && global_map_.data[x_tmp + y_tmp * width_] < 90)                     
                            {
                                count_thre++;
                            }
                        }
                    }
                    //如果计数器为3或4，检查16邻域是否有障碍物，如果有障碍物,设置noV为false
                    if(count_thre == 3 || count_thre == 4)
                    {
                        int noV = true;
                        for(int d = 0; d < 16; d++)
                        {
                            int x_tmp = i + d_x16[d];
                            int y_tmp = j + d_y16[d];
                            if(isInBorder(x_tmp, y_tmp))
                            {
                                if(global_map_.data[x_tmp + y_tmp * width_] >= 90) 
                                {
                                    noV = false;
                                    break;
                                }
                            }
                        }
                        //如果16邻域内没有障碍物，记录视点的索引坐标和实际坐标，并添加到视点列表中
                        if(noV) 
                        {
                            tmp_viewpoint.indexX = i;
                            tmp_viewpoint.indexY = j;
                            tmp_viewpoint.position = index2coord(i, j);
                            tmp_cen.viewpoint_fir.push_back(tmp_viewpoint);
                        }
                    }
                    //将当前栅格的x和y索引坐标存储到cellpoint结构体中
                    cellpoint.indexX = i;
                    cellpoint.indexY = j;
                    //设置栅格状态为未知区域
                    cellpoint.cellstate = -1;
                    //使用index2coord函数将栅格索引(i,j)转换为实际的物理坐标(x,y),并存储到cellpoint中。
                    cellpoint.position = index2coord(i, j);
                    count++;
                    //更新中心点位置
                    tmp_cen.centroidposition.x = (tmp_cen.centroidposition.x + cellpoint.position.x);
                    tmp_cen.centroidposition.y = (tmp_cen.centroidposition.y + cellpoint.position.y);
                    //将当前栅格信息添加到in_win_Unpoints_列表中
                    in_win_Unpoints_.push_back(cellpoint);
                }             
            }
        }
    }  
    //计算未知区域的几何中心，通过将之前累加的x和y坐标除以点的总数来获得平均位置，然后将连续坐标转换为网格索引
    if(count != 0)
    {
        tmp_cen.centroidposition.x = tmp_cen.centroidposition.x / count;
        tmp_cen.centroidposition.y = tmp_cen.centroidposition.y / count;
        tmp_cen.cenindex = coord2index(tmp_cen.centroidposition);
    } 
    //如果没有未知点(count=0)或未知点数量小于阈值，则直接返回true
    if(count == 0)
    {
        return true;
    }
    if(in_win_Unpoints_.size() <= unknown_num_thre_) 
    {
        return true;
    }
    //检查中心点是否在地图边界内
    if(isInBorder(tmp_cen.cenindex.x, tmp_cen.cenindex.y))
    {
        bool isUnKR = false;
        if(global_map_.data[tmp_cen.cenindex.x + tmp_cen.cenindex.y * width_] != -1)
        {
            //遍历8个方向，检查每个方向的栅格是否为未知区域，如果是则设置isUnKR为true
            for(int i = 0; i < 8; i++)
            {
                int x = tmp_cen.cenindex.x + d_x8[i];
                int y = tmp_cen.cenindex.y + d_y8[i];
                if(isInBorder(x,y))
                {
                    if(global_map_.data[x + y * width_] == -1)
                    {
                        isUnKR = true;
                        break;
                    }
                }
            }
            if(!isUnKR)
            {
                tmp_cen.isUnknown = false;
                return false;
            }
        } 
        else
            isUnKR = true;
        
        //如果确认是未知区域，则设置相关属性并检查视点数量。如果视点数量足够且与现有无路径中心点距离足够远，则将该中心点添加到centroids_列表中。
        if(isUnKR)
        {
            tmp_cen.isUnknown = true;
            tmp_cen.unknownnum = count;
            tmp_cen.windowsoder = winoder;
            if(tmp_cen.viewpoint_fir.size() >= 2) 
            {
                if(!no_path_cens_.empty())
                {
                    for(int i = 0; i < no_path_cens_.size(); i++)
                    {
                        if(abs(tmp_cen.cenindex.x - no_path_cens_[i].cenindex.x) < int(5.0 / resolution_)
                        && abs(tmp_cen.cenindex.y - no_path_cens_[i].cenindex.y) < int(5.0 / resolution_))
                        {

                        }
                        else
                        {
                            centroids_.push_back(tmp_cen);
                        }
                    }
                }
                else
                {
                    centroids_.push_back(tmp_cen);
                }
            }
            return true;
        }   
        return true; // debug warning
    }    
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("exploration_planning"), "Out of map!");
        return false;
    }
}

bool ExplorationPlanning::assessUnknownRegion(Centroid* choosecentroid)
{
    utils_ns::Index2 startpoint = choosecentroid->cenindex;
    int max_traver_degree = 0;
    int max_disgridnum = 0;
    double max_rvpathcost = 0.0;
    for(int i = 0; i < choosecentroid->viewpoint_fir.size(); i++)
    {
        utils_ns::Index2 endpoint;
        endpoint.x = choosecentroid->viewpoint_fir[i].indexX;
        endpoint.y = choosecentroid->viewpoint_fir[i].indexY;
        std::vector<Eigen::Vector2i> PassGrids;
        if(!utils_ns::Bresenham(endpoint.x, endpoint.y, startpoint.x, startpoint.y, PassGrids, global_map_))
        {
            continue;
        }
        else
        {
            utils_ns::Index2 startR, endV;
            startR.x = CONTXY2DISC(ego_position_.x - global_map_.info.origin.position.x, global_map_.info.resolution);
            startR.y = CONTXY2DISC(ego_position_.y - global_map_.info.origin.position.y, global_map_.info.resolution);
            endV.x = CONTXY2DISC(choosecentroid->viewpoint_fir[i].position.x - global_map_.info.origin.position.x, global_map_.info.resolution);
            endV.y = CONTXY2DISC(choosecentroid->viewpoint_fir[i].position.y - global_map_.info.origin.position.y, global_map_.info.resolution);
            int pathcostRV;
            std::vector<utils_ns::PointInt> tmppath;
            cal_cost_planner_->SearchforCost(startR.x, startR.y, endV.x, endV.y, global_map_, 
                                            global_map_.info.width, global_map_.info.height, 98, pathcostRV, tmppath);
            double pathdisRV = (double)(pathcostRV) / 10.0 * global_map_.info.resolution;
            if(pathdisRV > 10000.0 || pathdisRV < 6.0)
                continue;
            else
            {
                choosecentroid->viewpoint_fir[i].rv_path_cost = pathdisRV;
                if(pathdisRV > max_rvpathcost)
                    max_rvpathcost = pathdisRV;

                choosecentroid->viewpoint_fir[i].traver_degree 
                                    = global_map_.data[choosecentroid->viewpoint_fir[i].indexX + choosecentroid->viewpoint_fir[i].indexY * width_];
                choosecentroid->viewpoint_fir[i].disgridnum
                                    = PassGrids.size();
                if(choosecentroid->viewpoint_fir[i].traver_degree > max_traver_degree)
                    max_traver_degree = choosecentroid->viewpoint_fir[i].traver_degree;
                if(choosecentroid->viewpoint_fir[i].disgridnum > max_disgridnum)
                    max_disgridnum = choosecentroid->viewpoint_fir[i].disgridnum;

                choosecentroid->viewpoint_fin.push_back(choosecentroid->viewpoint_fir[i]);
            }
        }
    }
    float s_1;
    float s_2;
    float s_3;
    choosecentroid->evaluated_viewpoints.clear();
    for(int j = 0; j < choosecentroid->viewpoint_fin.size(); j++)
    {
        s_1 = max_rvpathcost > 0 ? float(choosecentroid->viewpoint_fin[j].rv_path_cost / max_rvpathcost) : 0;
        s_2 = max_traver_degree > 0 ? float(choosecentroid->viewpoint_fin[j].traver_degree / max_traver_degree) : 0;
        s_3 = max_disgridnum > 0 ? float(choosecentroid->viewpoint_fin[j].disgridnum / max_disgridnum) : 0;
        choosecentroid->viewpoint_fin[j].score = 1 * s_1 + s_2 + s_3;
        choosecentroid->evaluated_viewpoints.insert(make_pair(choosecentroid->viewpoint_fin[j].score, &choosecentroid->viewpoint_fin[j]));
    }
    if(choosecentroid->viewpoint_fin.empty())
        return false;
    else
        return true;
}

void ExplorationPlanning::purgeViewpoint(std::vector<utils_ns::ViewPoint> &purgedViewpoints, Centroid &tmpCentroid)
{
    int count = 0;
    bool emptyFlag = false;
    utils_ns::ViewPoint last_viewpoint;
    last_viewpoint.indexX = -1;
    last_viewpoint.indexY = -1;
    while(count < 6 && !emptyFlag)
    {
        if(!tmpCentroid.evaluated_viewpoints.empty())
        {
            auto current_viewpoint = tmpCentroid.evaluated_viewpoints.begin();
            if(abs(last_viewpoint.indexX - current_viewpoint->second->indexX) + abs(last_viewpoint.indexY - current_viewpoint->second->indexY) > 1)
            {
                current_viewpoint->second->fartherCentroidX = tmpCentroid.cenindex.x;
                current_viewpoint->second->fartherCentroidY = tmpCentroid.cenindex.y;
                purgedViewpoints.push_back(*current_viewpoint->second);
                last_viewpoint = *current_viewpoint->second;
                tmpCentroid.evaluated_viewpoints.erase(current_viewpoint);
            }
            else
                tmpCentroid.evaluated_viewpoints.erase(current_viewpoint);
            count++;
        }
        else
            emptyFlag = true;
    }
}

void ExplorationPlanning::execute()
{   
    bool pub_rviz = false;
    bool flag1 = false;

    // 如果机器人已经回家20次，则设置go_home_为true，输出无探索区域信息并返回
    if(go_home_count_ > 20)
        go_home_ = true;
    if(go_home_)
    {
        RCLCPP_INFO(rclcpp::get_logger("exploration_planning"),"There are almost no explorable unknown regions!");
        return;
    }

    // 用于设置和更新探索窗口(ExactWindow_)，定义了和限制机器人的探索窗口范围
    if(has_tra_map_ && has_map_) 
    {
        ExactWindow_.oriindx = CONTXY2DISC(traversibility_map_.info.origin.position.x - ori_x_, resolution_) + 3;
        ExactWindow_.oriindy = CONTXY2DISC(traversibility_map_.info.origin.position.y - ori_y_, resolution_) + 3;
        ExactWindow_.winwithd = traversibility_map_.info.width - 3;
        ExactWindow_.winheight = traversibility_map_.info.height - 3;
        if(min_left_index_.x > ExactWindow_.oriindx)
            min_left_index_.x = ExactWindow_.oriindx;
        if(min_left_index_.y > ExactWindow_.oriindy)
            min_left_index_.y = ExactWindow_.oriindy;
        if(max_right_index_.x < ExactWindow_.oriindx + ExactWindow_.winwithd)
            max_right_index_.x = ExactWindow_.oriindx + ExactWindow_.winwithd;
        if(max_right_index_.y < ExactWindow_.oriindy + ExactWindow_.winheight)
            max_right_index_.y = ExactWindow_.oriindy + ExactWindow_.winheight;
    }

    // 获取机器人位置
    getRobotPosition();

    if(!has_map_ || !has_robot_position_)
    {
       // RCLCPP_INFO(rclcpp::get_logger("exploration_planning"),"Waiting map or robot... \n");
    }
    else
    {
        auto begin_time = std::chrono::system_clock::now();

        update_cen_count_++;

        //定期更新中心点或在未找到中心点时尝试更新
        //如果当前窗口没有中心点,则扩展探索窗口大小
        if(update_cen_count_ % update_cen_thre_ == 0 || findCenIs_ == false)
        {

            findCenIs_ = getCentroid(ExactWindow_);
            if(!findCenIs_)
            {
                RCLCPP_INFO(rclcpp::get_logger("exploration_planning"),"The current window has no centroid, expand the exploration window size.");
                flag1 = true;
                ExactWindow_.oriindx = min_left_index_.x;
                ExactWindow_.oriindy = min_left_index_.y;
                ExactWindow_.winwithd = max_right_index_.x - min_left_index_.x;
                ExactWindow_.winheight = max_right_index_.y - min_left_index_.y;
                findCenIs_ = getCentroid(ExactWindow_);
            }
        }

        if(findCenIs_)
        // 如果找到中心点
        {
            flag1 = false;
            // 如果找到中心点，重置标志位,表示不需要扩展探索窗口
            pushcen_++;
            //增加推送中心点的计数器
            if(!last_centroids_.empty() && pushcen_ % 10 == 0)
            // 如果last_centroids_不为空且推送中心点的计数器为10的倍数
            {
                //遍历last_centroids_列表中的每个中心点，检查它们是否在未知区域中，则将其添加到all_outwin_centroids_列表中
                for(int i = 0; i < last_centroids_.size(); i++)
                {
                    if(global_map_.data[last_centroids_[i].cenindex.x + last_centroids_[i].cenindex.y * global_map_.info.width] == -1)
                    {
                        all_outwin_centroids_.push_back(last_centroids_[i]);
                    }
                }
                last_centroids_.clear();
                last_centroids_ = centroids_;
            }
            else if(last_centroids_.empty())
            {
                last_centroids_ = centroids_;
            }
            //设置findCenis为true表示找到了中心点，
            last_ex_state_.findCenis = true;
            last_ex_state_.findNearCenPathis = false;
        }
        else
        {
            //如果窗口外的中心点列表为空,输出警告信息表示几乎没有可探索的未知区域。
            if(all_outwin_centroids_.empty())
            {
                // ROS_INFO("DEBUG: all_outwin_centroids_ is empty");)
                RCLCPP_WARN(rclcpp::get_logger("exploration_planning"),"There are almost no explorable unknown regions!");
            }
            else
            {
                double min_rc_cost = 10000.0;
                //最小路径成本
                int record_index_cen = -1;
                //记录中心点索引
                std::vector<utils_ns::PointInt> min_dis_path;
                //最小距离路径
                nav_msgs::msg::Path exporation_path;  
                 //探索路径
                //如果未找到中心点且已找到通向邻近中心点的路径，上一个最近中心点位置的地图值小于95，机器人与上一个最近中心点的距离超过10
                if( last_ex_state_.findCenis == false 
                    && last_ex_state_.findNearCenPathis == true 
                    && global_map_.data[last_near_cen_.cenindex.x + last_near_cen_.cenindex.y * width_] < 95
                    && (abs(ego_position_.x - last_near_cen_.centroidposition.x) > 10 || abs(ego_position_.y - last_near_cen_.centroidposition.y) > 10))
                {
                    utils_ns::Index2 startR, endC;
                    //将机器人当前位置从连续坐标转换为离散网格索引。
                    startR.x = CONTXY2DISC(ego_position_.x - global_map_.info.origin.position.x, global_map_.info.resolution);
                    startR.y = CONTXY2DISC(ego_position_.y - global_map_.info.origin.position.y, global_map_.info.resolution);
                    endC.x = CONTXY2DISC(last_near_cen_.centroidposition.x - global_map_.info.origin.position.x, global_map_.info.resolution);
                    endC.y = CONTXY2DISC(last_near_cen_.centroidposition.y - global_map_.info.origin.position.y, global_map_.info.resolution);

                    int pathcostRC;
                    std::vector<utils_ns::PointInt> tmppath;
                    //使用cal_cost_planner_的SearchforNearCen函数搜索从机器人当前位置到上一个最近中心点的路径，并返回路径的成本和点坐标。
                    cal_cost_planner_->SearchforNearCen(startR.x, startR.y, endC.x, endC.y, global_map_, 
                                                    global_map_.info.width, global_map_.info.height, 95, pathcostRC, tmppath);
                    
                    exporation_path.header.frame_id = "map";
                    exporation_path.header.stamp = nh_->get_clock()->now();                           //clock->now();
                    exporation_path.poses.resize(tmppath.size());
                    //遍历路径点，将路径点转换为连续坐标，并存储在exporation_path中。
                    for (int i = 0; i < (int)tmppath.size(); i++)
                    {
                        if(height_ > tmppath[i].y && tmppath[i].y >= 0 && width_ > tmppath[i].x && tmppath[i].x >= 0)
                        {
                            double x = DISCXY2CONT(tmppath[i].x, resolution_) + ori_x_;
                            double y = DISCXY2CONT(tmppath[i].y, resolution_) + ori_y_;
                            exporation_path.poses[i].pose.position.x = x;
                            exporation_path.poses[i].pose.position.y = y; 
                            if(i + 1 < (int)tmppath.size())
                            {
                                double nx = DISCXY2CONT(tmppath[i + 1].x, resolution_) + ori_x_;
                                double ny = DISCXY2CONT(tmppath[i + 1].y, resolution_) + ori_y_;
                                exporation_path.poses[i].pose.orientation.w = atan2((ny - y),(nx -x));
                            } 
                            else  
                                exporation_path.poses[i].pose.orientation.w = exporation_path.poses[i - 1].pose.orientation.w;         
                        }
                    }
                }
                else 
                {
                    //遍历all_outwin_centroids_列表中的每个中心点，检查它们是否在机器人当前位置的minrange_范围内，如果不是，则跳过。
                    for(int j = 0; j < all_outwin_centroids_.size(); j++)
                    {
                        auto cur_cen = all_outwin_centroids_[j];
                        if(abs(ego_position_.x - cur_cen.centroidposition.x) < minrange_ 
                            && abs(ego_position_.y - cur_cen.centroidposition.y) < minrange_)
                            {                            
                                continue;
                            }
                        if(global_map_.data[cur_cen.cenindex.x + cur_cen.cenindex.y * global_map_.info.width] != -1)
                            continue;

                        utils_ns::Index2 startR, endC;
                        startR.x = CONTXY2DISC(ego_position_.x - global_map_.info.origin.position.x, global_map_.info.resolution);
                        startR.y = CONTXY2DISC(ego_position_.y - global_map_.info.origin.position.y, global_map_.info.resolution);
                        endC.x = CONTXY2DISC(all_outwin_centroids_[j].centroidposition.x - global_map_.info.origin.position.x, global_map_.info.resolution);
                        endC.y = CONTXY2DISC(all_outwin_centroids_[j].centroidposition.y - global_map_.info.origin.position.y, global_map_.info.resolution);
                        int pathcostRC;
                        std::vector<utils_ns::PointInt> tmppath;
                        cal_cost_planner_->SearchforNearCen(startR.x, startR.y, endC.x, endC.y, global_map_, 
                                                        global_map_.info.width, global_map_.info.height, 95, pathcostRC, tmppath);
                        double pathdisRC = (double)(pathcostRC) / 10.0 * global_map_.info.resolution;
                        //如果当前路径成本小于最小路径成本，则更新最小路径成本和记录的中心点索引，并存储路径点坐标。
                        if(min_rc_cost > pathdisRC)
                        {
                            min_rc_cost = pathdisRC;
                            record_index_cen = j;
                            min_dis_path = tmppath;
                        }
                    }
                    if(record_index_cen != -1)
                    {
                        Centroid min_dis_centroid = all_outwin_centroids_[record_index_cen];
            
                        choose_centroids_marker_->marker_.points.clear();
                        choose_centroids_marker_->marker_.points.push_back(min_dis_centroid.centroidposition);
                        choose_centroids_marker_->Publish();    

                        exporation_path.header.frame_id = "map";
                        exporation_path.header.stamp =nh_->get_clock()->now(); ;
                        exporation_path.poses.resize(min_dis_path.size());
                        for (int i = 0; i < (int)min_dis_path.size(); i++)
                        {
                            if(height_ > min_dis_path[i].y && min_dis_path[i].y >= 0 && width_ > min_dis_path[i].x && min_dis_path[i].x >= 0)
                            {
                                double x = DISCXY2CONT(min_dis_path[i].x, resolution_) + ori_x_;
                                double y = DISCXY2CONT(min_dis_path[i].y, resolution_) + ori_y_;
                                exporation_path.poses[i].pose.position.x = x;
                                exporation_path.poses[i].pose.position.y = y; 
                                if(i + 1 < (int)min_dis_path.size())
                                {
                                    double nx = DISCXY2CONT(min_dis_path[i + 1].x, resolution_) + ori_x_;
                                    double ny = DISCXY2CONT(min_dis_path[i + 1].y, resolution_) + ori_y_;
                                    exporation_path.poses[i].pose.orientation.w = atan2((ny - y),(nx -x));
                                } 
                                else  
                                    exporation_path.poses[i].pose.orientation.w = exporation_path.poses[i - 1].pose.orientation.w;         
                            }

                        }
                        //如果探索路径至少有两个点，则设置findCenis为false表示未找到中心点，findNearCenPathis为true表示已找到通向邻近中心点的路径，并记录最后一个最近中心点。
                        if(exporation_path.poses.size() >= 2)
                        {
                            last_ex_state_.findCenis = false;
                            last_ex_state_.findNearCenPathis = true;
                            last_near_cen_ = min_dis_centroid;
                        }                      
                    }
                }
                if(exporation_path.poses.size() >= 2)
                {
                    exploration_path_pub_->publish(exporation_path);
                    RCLCPP_INFO(rclcpp::get_logger("exploration_planning"),"Exploring nearby centroid... \n");  
                    flag1 = false;
                    go_home_count_ = 0;
                }
                else
                {
                    // ROS_INFO("DEBUG: There is no path to the nearby centroid !");
                    RCLCPP_WARN(rclcpp::get_logger("exploration_planning"),"There are almost no explorable unknown regions.");
                    go_home_count_++;
                }
            }
        }
        //如果centroids_列表中至少有一个中心点且findCenIs_为true，则执行以下代码。
        if(centroids_.size() > 0 && findCenIs_)
        {
            CandiCentroidPair target_centroid_pairs; 
            int findnCen = 0;

            std::vector<int> orders;
            if(centroids_.size() < 2)
            {
                orders.push_back(0);
                orders.push_back(1);
            }
            else
            {
                orders = getRouteOrder();
            }

            if(orders.size() <= 1)
            {
                RCLCPP_INFO(rclcpp::get_logger("exploration_planning"), "DEBUG: No route order ! ");
            }
            else
            {
                nav_msgs::msg::Path route_path;
                route_path.header.frame_id = "map";
                geometry_msgs::msg::PoseStamped ep;
                ep.pose.position = ego_position_;
                route_path.poses.push_back(ep); 
                for(int i = 1; i < orders.size(); i++)
                {
                    geometry_msgs::msg::PoseStamped cp;
                    cp.pose.position = centroids_[orders[i] - 1].centroidposition;
                    route_path.poses.push_back(cp); 
                }
                exploration_route_pub_->publish(route_path);

                for(int i = 1; i < orders.size(); i++)
                {
                    Centroid* current_cen= &centroids_[orders[i] - 1];
                    bool highIs = assessUnknownRegion(current_cen); // Prioritize exploring the high safety centroids that rank higher in the order. 
                    if(highIs)
                    {
                        findnCen++;
                    }
                    if(findnCen == 1 && highIs)
                    {
                        target_centroid_pairs.centroidone = centroids_[orders[i] - 1];
                        target_centroid_pairs.oneindexincentroids = orders[i] - 1;
                    }
                    if(findnCen == 2 && highIs)
                    {
                        target_centroid_pairs.centroidtwo = centroids_[orders[i] - 1];
                        target_centroid_pairs.twoindexincentroids = orders[i] - 1;
                        break;                       
                    }
                }
            }

            std::vector<utils_ns::ViewPoint> target_viewpoint1;
            std::vector<utils_ns::ViewPoint> target_viewpoint2;
            Centroid centroid1;
            Centroid centroid2;
            target_viewpoint1.clear();
            target_viewpoint2.clear();
            if(findnCen == 1)
            {
                target_centroid_pairs.centroidtwo = target_centroid_pairs.centroidone;
                target_centroid_pairs.twoindexincentroids = target_centroid_pairs.oneindexincentroids;
                first_two_cen_ = target_centroid_pairs;
                
                choose_centroids_marker_->marker_.points.clear();
                choose_centroids_marker_->marker_.points.push_back(first_two_cen_.centroidone.centroidposition);
                choose_centroids_marker_->marker_.points.push_back(first_two_cen_.centroidtwo.centroidposition);
                choose_centroids_marker_->Publish();

                centroid1 = first_two_cen_.centroidone;
                centroid2 = first_two_cen_.centroidtwo;

                // purgeViewpoint(target_viewpoint1, centroid1);
                // purgeViewpoint(target_viewpoint2, centroid2);
                target_viewpoint1.push_back(*centroid1.evaluated_viewpoints.begin()->second);
                auto endpt = centroid2.evaluated_viewpoints.end();
                endpt--;
                target_viewpoint2.push_back(*endpt->second);
                if(!target_viewpoint1.empty() && !target_viewpoint2.empty())
                    path_planner_.setViewPointSet(target_viewpoint1, target_viewpoint2, true, true);
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("exploration_planning"),"The evaluated centroid does not have a ahead viewpoint.");
                }

            }
            if(findnCen == 2)
            {
                first_two_cen_ = target_centroid_pairs;
              
                choose_centroids_marker_->marker_.points.clear();
                choose_centroids_marker_->marker_.points.push_back(first_two_cen_.centroidone.centroidposition);
                choose_centroids_marker_->marker_.points.push_back(first_two_cen_.centroidtwo.centroidposition);
                choose_centroids_marker_->Publish();

                centroid1 = first_two_cen_.centroidone;
                centroid2 = first_two_cen_.centroidtwo;
                target_viewpoint1.push_back(*centroid1.evaluated_viewpoints.begin()->second);
                target_viewpoint2.push_back(*centroid2.evaluated_viewpoints.begin()->second);
                if(!target_viewpoint1.empty() && !target_viewpoint2.empty())
                    path_planner_.setViewPointSet(target_viewpoint1, target_viewpoint2, true, true);
                else
                {
                   RCLCPP_INFO(rclcpp::get_logger("exploration_planning"),"The evaluated centroid does not have a ahead viewpoint.");
                }

            }
            else
            {
                if(orders.size() > 1) // There is no high safety region
                {
                    target_centroid_pairs.centroidone = centroids_[orders[1] - 1];
                    target_centroid_pairs.oneindexincentroids = orders[1] - 1;
                    target_centroid_pairs.centroidtwo = centroids_[orders[1] - 1];
                    target_centroid_pairs.twoindexincentroids = orders[1] - 1;
                    first_two_cen_ = target_centroid_pairs;
                    
                    choose_centroids_marker_->marker_.points.clear();
                    choose_centroids_marker_->marker_.points.push_back(first_two_cen_.centroidone.centroidposition);
                    choose_centroids_marker_->marker_.points.push_back(first_two_cen_.centroidtwo.centroidposition);
                    choose_centroids_marker_->Publish();
                    
                    centroid1 = first_two_cen_.centroidone;
                    centroid2 = first_two_cen_.centroidtwo;

                    path_planner_.setViewPointSet(centroid1.viewpoint_fir, centroid2.viewpoint_fir, false, false);
                }
                else
                {
                   RCLCPP_INFO(rclcpp::get_logger("exploration_planning"),"DEBUG: No centroid route !");
                }
            }

            choose_viewpoints_marker_->marker_.points.clear();
            if(!path_planner_.getViewPointSet1().empty())
            {
                for(const auto &it: path_planner_.getViewPointSet1())
                {
                    choose_viewpoints_marker_->marker_.points.push_back(it.position);
                }
                pub_rviz = true;
            }
            if(!path_planner_.getViewPointSet2().empty())
            {
                for(const auto &it: path_planner_.getViewPointSet2())
                {
                    choose_viewpoints_marker_->marker_.points.push_back(it.position);
                }
                pub_rviz = true;
            }
            if(pub_rviz)
                choose_viewpoints_marker_->Publish();

            utils_ns::Index2 centroidone, centroidtwo;
            centroidone.x = centroid1.cenindex.x;
            centroidone.y = centroid1.cenindex.y;
            centroidtwo.x = centroid2.cenindex.x;
            centroidtwo.y = centroid2.cenindex.y;
            
            if(use_viewpoint_plan_ && findnCen != 0)
            {
               centroidone.x = target_viewpoint1.begin()->indexX;
               centroidone.y = target_viewpoint1.begin()->indexY;
               centroidtwo.x = target_viewpoint2.begin()->indexX;
               centroidtwo.y = target_viewpoint2.begin()->indexY;
            }
            
            if(use_go_end_nearest_ && findnCen != 0 && !orders.empty())
            {
                int os = orders.size() - 1;
                auto end_nearest_cen = centroids_[orders[os] - 1];
                if(abs(end_nearest_cen.centroidposition.x - ego_position_.x) <= end_neacen_disthre_ 
                    && abs(end_nearest_cen.centroidposition.y - ego_position_.y) <= end_neacen_disthre_)
                {
                   if(abs(centroid1.centroidposition.x - ego_position_.x) / 
                      abs(end_nearest_cen.centroidposition.x - ego_position_.x) > end_cur_disrate_ ||
                      abs(centroid1.centroidposition.y - ego_position_.y) / 
                      abs(end_nearest_cen.centroidposition.y - ego_position_.y) > end_cur_disrate_ )
                    {
                        centroidone.x = end_nearest_cen.cenindex.x;
                        centroidone.y = end_nearest_cen.cenindex.y;
                        centroidtwo.x = centroid1.cenindex.x;
                        centroidtwo.y = centroid1.cenindex.y;
                    }               
                }
            }

            if(findnCen == 0) // the unknown region is medium safety level
            {
                centroidone.x = centroid1.viewpoint_fir.begin()->indexX;
                centroidone.y = centroid1.viewpoint_fir.begin()->indexY;
                int end_vp = centroid1.viewpoint_fir.size() - 1;
                centroidtwo.x = centroid1.viewpoint_fir[end_vp].indexX;
                centroidtwo.y = centroid1.viewpoint_fir[end_vp].indexY;
                //RCLCPP_WARN(rclcpp::get_logger("exploration_planning"),"Goal is vp");
            }            

            path_planner_.setCentroidPairIndex(centroidone, centroidtwo);
            path_planner_.setPlanMap(global_map_); 
            path_planner_.setRobotPosition(ego_position_);
            
            nav_msgs::msg::Path exporation_path;
            if(path_planner_.getExplorationPath(exporation_path))
            {
                path_planner_.finePath(exporation_path);
                exploration_path_pub_->publish(exporation_path);      
                go_home_count_ = 0;           
            }
            else
            {
                path_planner_.setCentroidPairIndex(centroidtwo, centroidone);
                if(path_planner_.getExplorationPath(exporation_path))
                {
                    path_planner_.finePath(exporation_path);
                    exploration_path_pub_->publish(exporation_path);
                    //RCLCPP_WARN(rclcpp::get_logger("exploration_planning"),"Get c2 path");     
                    go_home_count_ = 0;                 
                }
                else
                {
                    if(findnCen == 0)
                    {
                        no_path_cens_.push_back(centroid1);
                    }
                    RCLCPP_INFO(rclcpp::get_logger("exploration_planning"),"Replan.");
                    go_home_count_++;
                }
            }
        }
    
        auto end_time = std::chrono::system_clock::now();
        auto run_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time).count();
        run_time_s = (double)run_time / 1000;
        if(!flag1)
        {
            RCLCPP_WARN(rclcpp::get_logger("exploration_planning"),"Exploring ... ");
            // ROS_INFO("Route and path planning time is :  %0.2f  ms", run_time_s);
        }
    }
}
}

