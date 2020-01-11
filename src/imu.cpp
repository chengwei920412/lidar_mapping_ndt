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

    // 由于IMU频率比激光雷达高，offset_imu为在激光雷达两帧之间的imu积分，在新的激光雷达到达的时候就会被清除，避免长时间IMU积分产生累计误差
    offset_imu_roll += diff_imu_roll;
    offset_imu_pitch += diff_imu_pitch;
    offset_imu_yaw += diff_imu_yaw;

    // 在没有新的激光雷达数据来的时候，使用imu的角速度计更新角度，当新的imu数据来了之后，使用ndt的位姿来更新角度，避免IMU累计积分误差
    double currentpose_roll = previous_pose.roll + offset_imu_roll;
    double currentpose_pitch = previous_pose.pitch + offset_imu_pitch;
    double currentpose_yaw = previous_pose.yaw + offset_imu_yaw;

    // TODO 这里的IMU积分是完整的标准积分，由于自动驾驶车辆的没没有Y与Z的加速度，后续考虑变换为适应于车辆的IMU积分
    // 将imubody坐标系下面的线速度转换到世界坐标系下面
    // 先绕x轴旋转
    double accX1 = imu.linear_acceleration.x;
    double accY1 = std::cos(currentpose_roll) * imu.linear_acceleration.y - std::sin(currentpose_roll) * imu.linear_acceleration.z;
    double accZ1 = std::sin(currentpose_roll) * imu.linear_acceleration.y + std::cos(currentpose_roll) * imu.linear_acceleration.z;

    // 再绕y轴旋转
    double accX2 = std::cos(currentpose_pitch) * accX1 - std::sin(currentpose_pitch) * accZ1;
    double accY2 = accY1;
    double accZ2 = std::sin(currentpose_pitch) * accX1 + std::cos(currentpose_pitch) * accZ1;

    // 再绕z轴旋转
    double accX = std::cos(currentpose_yaw) * accX2 - std::sin(currentpose_yaw) * accY2;
    double accY = std::sin(currentpose_yaw) * accX2 + std::cos(currentpose_yaw) * accY2;
    double accZ = accZ2;

    // imu计算xyz方向上的偏移,初始速度用的imu_x为slam计算获得,后面再加上考虑加速度以及时间参数,获得较为准确的距离偏移
    // imu的速度值会通过slam进行修正,以避免累计误差，或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
    // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!
    offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
    offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
    offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

    // 在没有新的激光雷达数据来的时候，使用imu的加速度计更新速度，当新的imu数据来了之后，使用ndt的位置变化率来更新速度，避免IMU累计积分误差
    current_velocity_imu_x += accX * diff_time;
    current_velocity_imu_y += accY * diff_time;
    current_velocity_imu_z += accZ * diff_time;

    // guess_pose_imu 为在激光雷达到达间隔，通过IMU积分得到的位姿，用于给下一时刻的激光雷达提供初始值
    guess_pose_imu.x = previous_pose.x + offset_imu_x;
    guess_pose_imu.y = previous_pose.y + offset_imu_y;
    guess_pose_imu.z = previous_pose.z + offset_imu_z;
    guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
    guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
    guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

    previous_time = current_time;
}
}