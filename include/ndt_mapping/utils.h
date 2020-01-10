#ifndef NDT_UTILS_H
#define NDT_UTILS_H

#include <Eigen/Dense>
#include <ros/ros.h>

namespace ndt_mapping {
// ndt位姿结构体
struct pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    void init()
    {
        x = y = z = 0.0;
        roll = pitch = yaw = 0.0;
    }

    Eigen::Matrix4d rotateRPY()
    {
        Eigen::Translation3d tf_trans(x, y, z);
        Eigen::AngleAxisd rot_x(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z(yaw, Eigen::Vector3d::UnitZ());
        Eigen::Matrix4d mat = (tf_trans * rot_z * rot_y * rot_x).matrix();
        return mat;
    }
};

struct Param {
    bool is_create_pieceMap;
    bool is_create_planningMap;
    bool is_create_costMap;

    std::string pieceMap_folder;
    std::string planningMap_folder;
    std::string costMap_folder;

    Param()
    {
        is_create_costMap = false;
        is_create_pieceMap = false;
        is_create_planningMap = false;
    }
};

enum class MethodType {
    use_pcl = 0,
    use_cpu = 1,
    use_gpu = 2,
    use_omp = 3,
    use_gpu_ptr = 4,
};

static MethodType _method_type = MethodType::use_cpu; // 默认使用cpu运算

static bool _incremental_voxel_update = false; // ??????????

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, ndt_start, ndt_end,
    t5_start, t5_end;

static ros::Duration d_callback, d1, d2, d3, d4, d5;

void visualPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& first_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& second_cloud);

} // namespace ndt_mapping

#endif //NDT_UTILS_H