#include <ndt_mapping.h>

namespace ndt_mapping {

void LidarMapping::imuUpSideDown(const sensor_msgs::Imu::Ptr input)
{
    double input_roll, input_pitch, input_yaw;

    tf::Quaternion input_orientation;
    tf::quaternionMsgToTF(input->orientation, input_orientation);
    tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

    input->angular_velocity.x *= -1;
    input->angular_velocity.y *= -1;
    input->angular_velocity.z *= -1;

    input->linear_acceleration.x *= -1;
    input->linear_acceleration.y *= -1;
    input->linear_acceleration.z *= -1;

    input_roll *= -1;
    input_pitch *= -1;
    input_yaw *= -1;

    input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

void LidarMapping::imu_callback(const sensor_msgs::Imu::Ptr& input)
{
    if (_imu_upside_down) // _imu_upside_down指示是否进行imu的正负变换
        imuUpSideDown(input);

    const ros::Time current_time = input->header.stamp;
    static ros::Time previous_time = current_time;
    const double diff_time = (current_time - previous_time).toSec();

    // 解析imu消息,获得rpy
    double imu_roll, imu_pitch, imu_yaw;
    tf::Quaternion imu_orientation;
    tf::quaternionMsgToTF(input->orientation, imu_orientation);
    tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

    imu_roll = warpToPmPi(imu_roll); // 调整,防止超过PI(180°)  --保持在±180°内
    imu_pitch = warpToPmPi(imu_pitch);
    imu_yaw = warpToPmPi(imu_yaw);

    static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
    const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
    const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
    const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);
    //

    imu.header = input->header;
    imu.linear_acceleration.x = input->linear_acceleration.x;
    // imu.linear_acceleration.y = input->linear_acceleration.y;
    // imu.linear_acceleration.z = input->linear_acceleration.z;
    imu.linear_acceleration.y = 0;
    imu.linear_acceleration.z = 0;

    if (diff_time != 0) {
        imu.angular_velocity.x = diff_imu_roll / diff_time;
        imu.angular_velocity.y = diff_imu_pitch / diff_time;
        imu.angular_velocity.z = diff_imu_yaw / diff_time;
    } else {
        imu.angular_velocity.x = 0;
        imu.angular_velocity.y = 0;
        imu.angular_velocity.z = 0;
    }

    imu_calc(input->header.stamp);

    previous_time = current_time;
    previous_imu_roll = imu_roll;
    previous_imu_pitch = imu_pitch;
    previous_imu_yaw = imu_yaw;
}

void LidarMapping::imu_calc(ros::Time current_time)
{
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    double diff_imu_roll = imu.angular_velocity.x * diff_time;
    double diff_imu_pitch = imu.angular_velocity.y * diff_time;
    double diff_imu_yaw = imu.angular_velocity.z * diff_time;

    current_pose_imu.roll += diff_imu_roll;
    current_pose_imu.pitch += diff_imu_pitch;
    current_pose_imu.yaw += diff_imu_yaw;

    // 对imu由于不平衡造成的补偿问题,在这里解决
    // start1
    double accX1 = imu.linear_acceleration.x;
    double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y - std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
    double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y + std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

    double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
    double accY2 = accY1;
    double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

    double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
    double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
    double accZ = accZ2;
    // end1

    // imu计算xyz方向上的偏移,初始速度用的imu_x为slam计算获得,后面再加上考虑加速度以及时间参数,获得较为准确的距离偏移
    offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
    offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
    offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

    current_velocity_imu_x += accX * diff_time; // imu的速度值会通过slam进行修正,以避免累计误差
    current_velocity_imu_y += accY * diff_time; // 或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
    current_velocity_imu_z += accZ * diff_time; // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!@

    offset_imu_roll += diff_imu_roll;
    offset_imu_pitch += diff_imu_pitch;
    offset_imu_yaw += diff_imu_yaw;

    guess_pose_imu.x = previous_pose.x + offset_imu_x;
    guess_pose_imu.y = previous_pose.y + offset_imu_y;
    guess_pose_imu.z = previous_pose.z + offset_imu_z;
    guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
    guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
    guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

    previous_time = current_time;
}
}