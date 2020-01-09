#ifndef NDT_MAPPING_H
#define NDT_MAPPING_H

#include "ground_filter.hpp"

#include <ndt_cpu/NormalDistributionsTransform.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <sstream>
#include <thread>
#include <utils.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_omp_registration/ndt.h> // if USE_PCL_OPENMP

#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif

namespace ndt_mapping {

using PointI = pcl::PointXYZ;
using Point = pcl::PointXYZ;

class LidarMapping {
private:
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;
    double param_tf_timeout;
    std::string param_base_frame;
    std::string param_laser_frame;
    Eigen::Matrix4f tf_btol, tf_ltob; // base_link -> laser_link

    pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom;
    pose current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom;
    pose ndt_pose, localizer_pose;
    pose added_pose; // added_pose记录点云加入地图时候的位置  // 初始设为0即可,因为第一帧如论如何也要加入地图的

    ros::Time current_scan_time;
    ros::Time previous_scan_time;
    ros::Duration scan_duration;

    // 定义Publisher
    ros::Publisher debug_map_pub;
    ros::Publisher matching_map_pub; // 地图发布
    ros::Publisher refiltered_map_pub;
    ros::Publisher current_pose_pub; // 位置发布

    ros::Subscriber points_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber odom_sub;
    geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

    ros::Publisher ndt_stat_pub;
    std_msgs::Bool ndt_stat_msg; // 确认是否是ndt的第一帧图像 bool类型

    // 设置变量,用以接受ndt配准后的参数
    double fitness_score;
    bool has_converged;
    int final_num_iteration;
    double transformation_probability;

    // 设置变量,用以接受imu和odom消息
    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;

    MethodType _method_type;

    // 定义各种差异值(两次采集数据之间的差异,包括点云位置差异,imu差异,odom差异,imu-odom差异)
    double diff;
    double diff_x, diff_y, diff_z, diff_yaw; // current_pose - previous_pose // 定义两帧点云差异值 --以确定是否更新点云等
    double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
    double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
    double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
        offset_imu_odom_yaw;

    // 定义速度值 --包括实际速度值,和imu取到的速度值
    double current_velocity_x;
    double current_velocity_y;
    double current_velocity_z;

    double current_velocity_imu_x;
    double current_velocity_imu_y;
    double current_velocity_imu_z;

    pcl::NormalDistributionsTransform<Point, Point> pcl_ndt;
    cpu::NormalDistributionsTransform<Point, Point> cpu_ndt; // cpu方式  --可以直接调用吗??可以
#ifdef CUDA_FOUND
    gpu::GNormalDistributionsTransform gpu_ndt;
    // TODO:此处增加共享内存方式的gpu_ndt_ptr
    // std::shared_ptr<gpu::GNormalDistributionsTransform> gpu_ndt = std::make_shared<gpu::GNormalDistributionsTransform>();
#endif
    //#ifdef USE_PCL_OPENMP  // 待查证使用方法
    //		static pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> omp_ndt;
    //#endif
    pcl_omp::NormalDistributionsTransform<Point, Point> omp_ndt;

    // Default values  // 公共ndt参数设置
    int max_iter; // Maximum iterations
    float ndt_res; // Resolution
    double step_size; // Step size
    double trans_eps; // Transformation epsilon

    // Leaf size of VoxelGrid filter.   ---该处定义的是一个默认的栅格大小
    double voxel_leaf_size;

    Eigen::Matrix4f gnss_transform; // 保存GPS信号的变量

    double min_scan_range; // min和max用于第一次进行点云过滤(截取两个同心圆内的)
    double max_scan_range;
    double min_add_scan_shift; // 定义将点云添加到locaMap里的最小间隔值  --应该是添加到localMap吧??

    bool initial_scan_loaded;

    // 重要变量参数 :: 用以指示是否使用imu,是否使用odom
    bool _use_imu;
    bool _use_odom;
    bool _imu_upside_down; // 用以解决坐标系方向(正负变换)问题 (比如x变更为-x等)

    std::string _imu_topic; // 定义imu消息的topic
    std::string _odom_topic;
    std::string _lidar_topic;

    utils::RayGroundFilter filter;
    pcl::PointCloud<Point> global_map_no_ground;
    ros::Publisher pub_global_map_no_ground;
    pcl::PointCloud<Point> global_map_for_costmap;
    ros::Publisher pub_global_map_for_costmap;

    pthread_t thread;
    Param* param;

    // global map params
    bool is_publish_map_full;
    pcl::PointCloud<Point> map; // 此处定义地图  --用于ndt匹配
    pcl::PointCloud<Point> global_map; // 此处为构建打全局地图
    // pcl::PointCloud<pcl::PointXYZI> target_map;
    double param_global_voxel_leafsize;
    double param_min_update_target_map;
    double param_extract_length;
    double param_extract_width;
    bool param_visualize;

    // piece map params
    bool param_if_create_piece_map;
    std::string param_piece_map_folder;
    int param_piecemap_size;
    double param_piece_map_voxel_size;
    pcl::PointCloud<Point> piece_map_container;
    // bool is_first_piece_map;
    // int before_x_index, before_y_index;
    // int current_x_index, current_y_index;

    // planning map params
    bool is_publish_map_for_planning;
    std::string param_planning_map_folder;
    int param_planning_piece_size;
    double param_planning_voxel_size;
    pcl::PointCloud<PointI> whole_planning_map;

    // cost map params
    bool is_publish_map_for_costmap;
    int param_cost_piece_size;
    double param_costmap_voxel_size;
    std::string param_cost_map_folder;
    pcl::PointCloud<pcl::PointXYZ> whole_cost_map;

    double lidar_height;

    static std::queue<pcl::PointCloud<Point>> pieceMap_queue;
    static std::queue<pcl::PointCloud<Point>> planningMap_queue;
    static std::queue<pcl::PointCloud<Point>> costMap_queue;

public:
    LidarMapping();

    ~LidarMapping() {}

    static void* thread_map_saver(void* params);

    void param_initial(ros::NodeHandle& nh, ros::NodeHandle& private_handle);

    void imu_odom_calc(ros::Time current_time);
    void imu_calc(ros::Time current_time);
    void odom_calc(ros::Time current_time);

    double warpToPm(double a_num, const double a_max);
    double warpToPmPi(double a_angle_rad);
    void imuUpSideDown(const sensor_msgs::Imu::Ptr input);
    double calcDiffForRadian(const double lhs_rad, const double rhs_rad);

    void imu_callback(const sensor_msgs::Imu::Ptr& input);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& input);

    void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
    // piece map functions

    // 入口函数
    void run(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    void clipCloudbyMinAndMaxDis(const pcl::PointCloud<Point>::Ptr& input_cloud, pcl::PointCloud<Point>::Ptr& output_cloud, const double& min_dis, const double& max_dis);
};

} // namespace ndt_mapping

#endif //NDT_MAPPING_H