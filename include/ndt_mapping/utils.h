#ifndef NDT_UTILS_H
#define NDT_UTILS_H

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <string>

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

void visualPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& first_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& second_cloud);

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
class timer_ms {
public:
    timer_ms()
    {
        beginTime = high_resolution_clock::now();
    }
    timer_ms(std::string s)
    {
        name = s;
        beginTime = high_resolution_clock::now();
    }
    ~timer_ms() {}
    void print_time()
    {
        high_resolution_clock::time_point endTime = high_resolution_clock::now();
        milliseconds timeInterval = std::chrono::duration_cast<milliseconds>(endTime - beginTime);
        std::cout << name << " consuming " << timeInterval.count() << "ms\n";
    }

private:
    high_resolution_clock::time_point beginTime;
    std::string name = "Timer";
};

} // namespace ndt_mapping

#endif //NDT_UTILS_H