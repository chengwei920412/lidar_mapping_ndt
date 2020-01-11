#include <ndt_mapping.h>

namespace ndt_mapping {

void LidarMapping::odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
    odom = *input;
    odom_calc(input->header.stamp);
}

void LidarMapping::odom_calc(ros::Time current_time)
{
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
    double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
    double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

    offset_odom_roll += diff_odom_roll;
    offset_odom_pitch += diff_odom_pitch;
    offset_odom_yaw += diff_odom_yaw;

    // double currentpose_roll = previous_pose.roll + offset_odom_roll;
    double currentpose_pitch = previous_pose.pitch + offset_odom_pitch;
    double currentpose_yaw = previous_pose.yaw + offset_odom_yaw;

    double diff_distance = odom.twist.twist.linear.x * diff_time;
    offset_odom_x += diff_distance * cos(-currentpose_pitch) * cos(currentpose_yaw);
    offset_odom_y += diff_distance * cos(-currentpose_pitch) * sin(currentpose_yaw);
    offset_odom_z += diff_distance * sin(-currentpose_pitch);

    guess_pose_odom.x = previous_pose.x + offset_odom_x;
    guess_pose_odom.y = previous_pose.y + offset_odom_y;
    guess_pose_odom.z = previous_pose.z + offset_odom_z;
    guess_pose_odom.roll = previous_pose.roll + offset_odom_roll;
    guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
    guess_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

    previous_time = current_time;
}

void LidarMapping::imu_odom_calc(ros::Time current_time)
{
    // 利用imu角速度较准确，轮速计的线速度较准确，进行里程计的积分
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    // imu信息处理,计算 -- imu只使用陀螺仪,即 只输出转角信息roll,pitch,yaw
    double diff_imu_roll = imu.angular_velocity.x * diff_time;
    double diff_imu_pitch = imu.angular_velocity.y * diff_time;
    double diff_imu_yaw = imu.angular_velocity.z * diff_time;

    offset_imu_odom_roll += diff_imu_roll;
    offset_imu_odom_pitch += diff_imu_pitch;
    offset_imu_odom_yaw += diff_imu_yaw;

    // double currentpose_roll = previous_pose.roll + offset_imu_odom_roll;
    double currentpose_pitch = previous_pose.pitch + offset_imu_odom_pitch;
    double currentpose_yaw = previous_pose.yaw + offset_imu_odom_yaw;

    // odom信息处理,计算 -- xyz移动距离的计算,融合odom的速度(位移)信息和imu的转角信息
    double diff_distance = odom.twist.twist.linear.x * diff_time;
    offset_imu_odom_x += diff_distance * cos(-currentpose_pitch) * cos(currentpose_yaw);
    offset_imu_odom_y += diff_distance * cos(-currentpose_pitch) * sin(currentpose_yaw);
    offset_imu_odom_z += diff_distance * sin(-currentpose_pitch);

    // ==> 最终的目的是融合imu和odom输出一个guess_pose
    // 注:guess_pose是在previous_pose基础上叠加一个offset,包括xyz的和rpy
    // xyz的offset需要融合imu的转角和odom的速度(位移)
    // rpy的offset直接采用imu的rpy偏差值
    guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
    guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
    guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
    guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
    guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
    guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

    previous_time = current_time;
}
}