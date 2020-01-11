#include <ndt_mapping.h>

#include <iostream>

namespace ndt_mapping {

void LidarMapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<Point>::Ptr input_cloud(new pcl::PointCloud<Point>()), scan_ptr(new pcl::PointCloud<Point>());

    current_scan_time = input->header.stamp;
    // ndt start time recorder
    ndt_start = ros::Time::now();

    pcl::fromROSMsg(*input, *input_cloud);

    // 截取圆环内的点云
    clipCloudbyMinAndMaxDis(input_cloud, scan_ptr, min_scan_range, max_scan_range);

    // 将输入的点云从激光雷达坐标系转换到以后轮中心的车辆坐标系
    // 点云原本是以激光雷达自身坐标系为原点
    // 将第一帧点云转换到以车辆后轮中心坐标系为原点，并添加到地图中
    // 地图的世界坐标系原点位于车辆后轴中心
    pcl::PointCloud<Point>::Ptr transformed_scan_ptr(new pcl::PointCloud<Point>());
    if (!initial_scan_loaded) {
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_base_laser);
        map += *transformed_scan_ptr;
        global_map += *transformed_scan_ptr;
    }

    // 匹配之前做去地面处理
    // filter.setIfClipHeight(false);
    // filter.setMinDistance(1.0);
    // pcl::PointCloud<Point>::Ptr local_no_ground(new pcl::PointCloud<Point>());
    // pcl::PointCloud<Point>::Ptr to_globalmap_for_costmap(new pcl::PointCloud<Point>());
    // filter.convert(scan_ptr, local_no_ground);

    // 对scan_ptr进行降采样
    pcl::PointCloud<Point>::Ptr filtered_scan_ptr(new pcl::PointCloud<Point>());
    downSampleFilter(scan_ptr, filtered_scan_ptr, voxel_leaf_size);

    // 设置NDT source点云
    if (_method_type == MethodType::use_pcl) {
        pcl_ndt.setInputSource(filtered_scan_ptr);
    } else if (_method_type == MethodType::use_cpu) {
        cpu_ndt.setInputSource(filtered_scan_ptr);
        // cpu_ndt.setInputSource(scan_ptr);

    }
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::use_gpu) {
        gpu_ndt.setInputSource(filtered_scan_ptr);
    }
#endif
    else if (_method_type == MethodType::use_omp) {
        omp_ndt.setInputSource(filtered_scan_ptr);
    } else {
        ROS_ERROR("Please Define _method_type to conduct NDT");
        exit(1);
    }

    // 设置NDT target点云
    pcl::PointCloud<Point>::Ptr map_ptr = boost::make_shared<pcl::PointCloud<Point>>(map);
    if (!initial_scan_loaded) {
        ROS_INFO("add first map");
        if (_method_type == MethodType::use_pcl) {
            pcl_ndt.setInputTarget(map_ptr);
        } else if (_method_type == MethodType::use_cpu) {
            cpu_ndt.setInputTarget(map_ptr);
            // cpu_ndt.setInputTarget(transformed_scan_ptr);
        }
#ifdef CUDA_FOUND
        else if (_method_type == MethodType::use_gpu) {
            gpu_ndt.setInputTarget(map_ptr);
        }
#endif
        else if (_method_type == MethodType::use_omp) {
            omp_ndt.setInputTarget(map_ptr);
        }
        initial_scan_loaded = true;
    }

    // 根据是否使用imu和odom,按照不同方式更新guess_pose(xyz,or/and rpy)
    pose guess_pose_for_ndt;
    if (_use_imu && _use_odom) {
        imu_odom_calc(current_scan_time);
        guess_pose_for_ndt = guess_pose_imu_odom;
    } else if (_use_imu && !_use_odom) {
        imu_calc(current_scan_time);
        guess_pose_for_ndt = guess_pose_imu;
    } else if (!_use_imu && _use_odom) {
        odom_calc(current_scan_time);
        guess_pose_for_ndt = guess_pose_odom;
    } else {
        guess_pose.x = previous_pose.x + diff_x; // 初始时diff_x等都为0
        guess_pose.y = previous_pose.y + diff_y;
        guess_pose.z = previous_pose.z + diff_z;
        guess_pose.roll = previous_pose.roll;
        guess_pose.pitch = previous_pose.pitch;
        guess_pose.yaw = previous_pose.yaw + diff_yaw;

        guess_pose_for_ndt = guess_pose;
    }

    // 根据guess_pose_for_ndt 来计算初始变换矩阵guess_init
    Eigen::AngleAxisf init_rotation_x(static_cast<const float&>(guess_pose_for_ndt.roll), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(static_cast<const float&>(guess_pose_for_ndt.pitch), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(static_cast<const float&>(guess_pose_for_ndt.yaw), Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(static_cast<const float&>(guess_pose_for_ndt.x),
        static_cast<const float&>(guess_pose_for_ndt.y),
        static_cast<const float&>(guess_pose_for_ndt.z));

    //这里是初始位姿估计，不过由于guess_pose_for_ndt位姿估计是车辆后轴坐标系在世界坐标系map的位姿，还需要转化为激光雷达坐标系在世界坐标系map下面的位姿
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_base_laser;

    // 用以保存ndt转换后的点云,align参数

    pcl::PointCloud<Point>::Ptr output_cloud(new pcl::PointCloud<Point>);
    if (_method_type == MethodType::use_pcl) {
        pcl_ndt.align(*output_cloud, init_guess); // pcl::aligin 需传入转换后的点云(容器),估计变换
        fitness_score = pcl_ndt.getFitnessScore();
        t_localizer = pcl_ndt.getFinalTransformation(); // t_localizer为ndt变换得到的最终变换矩阵(即source和target之间的变换)
        has_converged = pcl_ndt.hasConverged();
        final_num_iteration = pcl_ndt.getFinalNumIteration();
        transformation_probability = pcl_ndt.getTransformationProbability();
    } else if (_method_type == MethodType::use_cpu) {
        cpu_ndt.align(init_guess); // cpu::align 只需要传入估计变换 --建图的时候传入估计变换,定位matching的时候传入空的单位Eigen
        fitness_score = cpu_ndt.getFitnessScore();
        t_localizer = cpu_ndt.getFinalTransformation();
        has_converged = cpu_ndt.hasConverged();
        final_num_iteration = cpu_ndt.getFinalNumIteration();
    }
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::use_gpu) {
        gpu_ndt.align(init_guess); // ndt_gpu库的align,不传出配准后的点云 ---用法同cpu_ndt
        fitness_score = gpu_ndt.getFitnessScore();
        t_localizer = gpu_ndt.getFinalTransformation();
        has_converged = gpu_ndt.hasConverged();
        final_num_iteration = gpu_ndt.getFinalNumIteration();
    }
#endif
    else if (_method_type == MethodType::use_omp) {
        omp_ndt.align(*output_cloud, init_guess); // omp_ndt.align用法同pcl::ndt
        fitness_score = omp_ndt.getFitnessScore();
        t_localizer = omp_ndt.getFinalTransformation();
        has_converged = omp_ndt.hasConverged();
        final_num_iteration = omp_ndt.getFinalNumIteration();
    }

    ndt_end = ros::Time::now();

    if (final_num_iteration > 20) {
        ROS_WARN("num of iteration > 20 !, abandon this align");
        return;
    }
    // t_localizer是激光雷达在世界坐标系map下面的坐标
    // tf_laser_base是车辆坐标系在世界坐标系map下面的坐标
    t_base_link = t_localizer * tf_laser_base;

    tf::Matrix3x3 mat_b;
    mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
        static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
        static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
        static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
        static_cast<double>(t_base_link(2, 2)));

    // 更新全局下的坐标
    ndt_pose.x = t_base_link(0, 3);
    ndt_pose.y = t_base_link(1, 3);
    ndt_pose.z = t_base_link(2, 3);
    mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

    // current_pose
    current_pose.x = ndt_pose.x;
    current_pose.y = ndt_pose.y;
    current_pose.z = ndt_pose.z;
    current_pose.roll = ndt_pose.roll;
    current_pose.pitch = ndt_pose.pitch;
    current_pose.yaw = ndt_pose.yaw;

    // 发布tf变换： map到base之间的变换
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    tf::Quaternion q;
    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", param_base_frame));

    updateParams();

    // timer_ms t1("update_map");
    // 按照步长,更新全局地图map
    // 保存 piece_map， planning_map, cost_map
    double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
    if (shift >= min_add_scan_shift) {
        added_pose.x = current_pose.x;
        added_pose.y = current_pose.y;
        added_pose.z = current_pose.z;
        added_pose.roll = current_pose.roll;
        added_pose.pitch = current_pose.pitch;
        added_pose.yaw = current_pose.yaw;

        pcl::PointCloud<Point>::Ptr to_map(new pcl::PointCloud<Point>());
        downSampleFilter(transformed_scan_ptr, to_map, voxel_leaf_size);
        map += *to_map;

        // piece_map
        if (param_if_create_piece_map) {
            savePieceMap(transformed_scan_ptr);
        }

        // 发布用于定义可行驶区域,提取轨迹点的地图
        // 该地图保留地面,且高度仅保留地面以上30公分左右
        if (is_publish_map_for_planning) {
            pcl::PointCloud<PointI>::Ptr in_cloud_raw(new pcl::PointCloud<PointI>());
            pcl::fromROSMsg(*input, *in_cloud_raw);
            saveMapForPlanning(in_cloud_raw);
        }

        // 发布用于生成全局costmap的地图,地面过滤,高度截取到地面以上2m
        if (is_publish_map_for_costmap) {
            saveMapForCostmap(scan_ptr);
        }

        // 发布地图消息
        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
        refiltered_map_pub.publish(*map_msg_ptr);
    }

    // t1.print_time();

    // timer_ms t2("set_target_map");
    // 如果将target_map作为target, setInputTarget 耗时稳定在70～85ms左右,生成target_map耗时在13～23ms左右
    // 如果将整个map作为target, setInputTarget 耗时随着地图不断增加，在地图后期较大的时候可能达到400ms
    if (shift >= param_min_update_target_map) {
        // timer_ms t3("generate target map");
        pcl::PointCloud<Point>::Ptr target_map(new pcl::PointCloud<Point>());
        target_map->points.clear();
        for (auto point : map.points) {
            double diff_xx = abs(point.x - current_pose.x), diff_yy = abs(point.y - current_pose.y);
            if (diff_xx > max_scan_range || diff_yy > max_scan_range) {
                continue;
            }
            double len = std::sqrt(std::pow(diff_xx, 2) + std::pow(diff_yy, 2));
            if (len <= max_scan_range) {
                target_map->points.push_back(point);
            }
        }
        // t3.print_time();

        // timer_ms t4("add target map");
        if (_method_type == MethodType::use_pcl)
            pcl_ndt.setInputTarget(target_map); // 注意:此时加入的target:map_ptr并不包括刚加入点云的transformed_scan_ptr
        else if (_method_type == MethodType::use_cpu) // 只是说map更新了,因此target也要更新,不要落后太多
        {
            if (_incremental_voxel_update)
                cpu_ndt.updateVoxelGrid(transformed_scan_ptr);
            else
                cpu_ndt.setInputTarget(target_map);
        } else if (_method_type == MethodType::use_gpu)
#ifdef CUDA_FOUND
            gpu_ndt.setInputTarget(target_map);
#else
            ROS_ERROR("set to ndt_gpu, but no cuda found!");
#endif
        else if (_method_type == MethodType::use_omp)
            omp_ndt.setInputTarget(target_map);
        // t4.print_time();
    }

    // t2.print_time();
    // 发布ndt定位位置
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = current_scan_time;
    current_pose_msg.pose.position.x = current_pose.x;
    current_pose_msg.pose.position.y = current_pose.y;
    current_pose_msg.pose.position.z = current_pose.z;
    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    current_pose_msg.pose.orientation.x = q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();

    current_pose_pub.publish(current_pose_msg);

    // for debug
    if (param_visualize) {
        std::cout << "---------------------------------------------" << std::endl;
        std::cout << "Sequence number: " << input->header.seq << std::endl;
        std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
        std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
        std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
        std::cout << "global_map: " << map.points.size() << " points." << std::endl;
        std::cout << "NDT Used Time: " << (ndt_end - ndt_start) << "s" << std::endl;
        std::cout << "NDT has converged: " << has_converged << std::endl;
        std::cout << "Fitness score: " << fitness_score << std::endl;
        std::cout << "Number of iteration: " << final_num_iteration << std::endl;
        std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
        std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
                  << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
        std::cout << "Transformation Matrix:" << std::endl;
        std::cout << t_localizer << std::endl;
        // std::cout << "Transformation Matrix: (base_link)" << std::endl;
        // std::cout << t_base_link << std::endl;
        std::cout << "shift: " << shift << std::endl;

        std::cout << "---------------------------------------------" << std::endl;
    }
}

void LidarMapping::updateParams()
{
    // 根据current和previous两帧之间的scantime,以及两帧之间的位置,计算两帧之间的变化
    double secs = (current_scan_time - previous_scan_time).toSec();

    // 计算两个激光帧之间的偏移量
    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    // 计算两帧之间的速度
    current_velocity_x = diff_x / secs;
    current_velocity_y = diff_y / secs;
    current_velocity_z = diff_z / secs;

    // 修正imu速度
    current_velocity_imu_x = current_velocity_x;
    current_velocity_imu_y = current_velocity_y;
    current_velocity_imu_z = current_velocity_z;

    // 更新上一时刻的参数
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    previous_scan_time.sec = current_scan_time.sec;
    previous_scan_time.nsec = current_scan_time.nsec;

    // 清空里程计积分
    offset_imu_x = 0.0;
    offset_imu_y = 0.0;
    offset_imu_z = 0.0;
    offset_imu_roll = 0.0;
    offset_imu_pitch = 0.0;
    offset_imu_yaw = 0.0;

    offset_odom_x = 0.0;
    offset_odom_y = 0.0;
    offset_odom_z = 0.0;
    offset_odom_roll = 0.0;
    offset_odom_pitch = 0.0;
    offset_odom_yaw = 0.0;

    offset_imu_odom_x = 0.0;
    offset_imu_odom_y = 0.0;
    offset_imu_odom_z = 0.0;
    offset_imu_odom_roll = 0.0;
    offset_imu_odom_pitch = 0.0;
    offset_imu_odom_yaw = 0.0;
}

} // namespace FAST_NDT
