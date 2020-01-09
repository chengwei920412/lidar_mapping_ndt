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
    pcl::PointCloud<Point>::Ptr transformed_scan_ptr(new pcl::PointCloud<Point>());
    if (!initial_scan_loaded) {
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
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
    pcl::VoxelGrid<Point> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    // 设置NDT source点云
    if (_method_type == MethodType::use_pcl) {
        pcl_ndt.setInputSource(filtered_scan_ptr);
    } else if (_method_type == MethodType::use_cpu) {
        cpu_ndt.setInputSource(filtered_scan_ptr);
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

    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;

    tf::Quaternion q;

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());

    static tf::TransformBroadcaster br;

    tf::Transform transform;

    pcl::PointCloud<PointI>::Ptr in_cloud_xyzp(new pcl::PointCloud<PointI>());

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

    t_base_link = t_localizer * tf_ltob;

    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

    tf::Matrix3x3 mat_l, mat_b; // 用以根据齐次坐标下的旋转变换,来求rpy转换角度
    mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
        static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
        static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
        static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
        static_cast<double>(t_localizer(2, 2)));

    mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
        static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
        static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
        static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
        static_cast<double>(t_base_link(2, 2)));

    // Update localizer_pose.  // 更新局部下的坐标
    localizer_pose.x = t_localizer(0, 3);
    localizer_pose.y = t_localizer(1, 3);
    localizer_pose.z = t_localizer(2, 3);
    mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

    // Update ndt_pose.  // 更新全局下的坐标
    ndt_pose.x = t_base_link(0, 3);
    ndt_pose.y = t_base_link(1, 3);
    ndt_pose.z = t_base_link(2, 3);
    mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);
    // current_pose
    current_pose.x = ndt_pose.x;
    current_pose.y = ndt_pose.y;
    current_pose.z = ndt_pose.z;
    current_pose.roll = ndt_pose.roll;
    current_pose.pitch = ndt_pose.pitch;
    current_pose.yaw = ndt_pose.yaw;

    transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", param_base_frame));

    //		根据current和previous两帧之间的scantime,以及两帧之间的位置,计算两帧之间的变化
    scan_duration = current_scan_time - previous_scan_time;
    double secs = scan_duration.toSec();

    // Calculate the offset (curren_pos - previous_pos)
    // *****************************************
    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    current_velocity_x = diff_x / secs;
    current_velocity_y = diff_y / secs;
    current_velocity_z = diff_z / secs;

    current_pose_imu.x = current_pose.x;
    current_pose_imu.y = current_pose.y;
    current_pose_imu.z = current_pose.z;
    current_pose_imu.roll = current_pose.roll;
    current_pose_imu.pitch = current_pose.pitch;
    current_pose_imu.yaw = current_pose.yaw;

    current_pose_odom.x = current_pose.x;
    current_pose_odom.y = current_pose.y;
    current_pose_odom.z = current_pose.z;
    current_pose_odom.roll = current_pose.roll;
    current_pose_odom.pitch = current_pose.pitch;
    current_pose_odom.yaw = current_pose.yaw;

    current_pose_imu_odom.x = current_pose.x;
    current_pose_imu_odom.y = current_pose.y;
    current_pose_imu_odom.z = current_pose.z;
    current_pose_imu_odom.roll = current_pose.roll;
    current_pose_imu_odom.pitch = current_pose.pitch;
    current_pose_imu_odom.yaw = current_pose.yaw;

    current_velocity_imu_x = current_velocity_x; // 修正imu速度
    current_velocity_imu_y = current_velocity_y;
    current_velocity_imu_z = current_velocity_z;
    // ************************************************

    // Update position and posture. current_pos -> previous_pos
    // -----------------------------------------------
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    previous_scan_time.sec = current_scan_time.sec;
    previous_scan_time.nsec = current_scan_time.nsec;

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
    // ------------------------------------------------

    // Calculate the shift between added_pos and current_pos // 以确定是否更新全局地图
    // added_pose将一直定位于localMap的原点
    // ###############################################################################

    ros::Time test_time_5 = ros::Time::now(); // TODO:

    // 发布用于定义可行驶区域,提取轨迹点的地图 // 该地图保留地面,且高度仅保留地面以上30公分左右
    pcl::PointCloud<PointI>::Ptr transformed_incloud(new pcl::PointCloud<PointI>());
    if (is_publish_map_for_planning) {
        // 输入点云部分截取
        pcl::PointCloud<PointI>::Ptr in_cloud_raw(new pcl::PointCloud<PointI>());
        pcl::fromROSMsg(*input, *in_cloud_raw);
        pcl::PointIndices indices;
        for (size_t i = 0; i < in_cloud_raw->points.size(); i++) {
            double x = in_cloud_raw->points[i].x;
            double y = in_cloud_raw->points[i].y;
            double z = in_cloud_raw->points[i].z;
            double dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

            if (dist > 1.0 && dist < 50.0 && z < -1.3)
                indices.indices.push_back(i);
        }
        pcl::ExtractIndices<PointI> extractor;
        extractor.setInputCloud(in_cloud_raw);
        extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        extractor.setNegative(false); // false to save the indices
        extractor.filter(*in_cloud_xyzp);

        // TODO:: 下采样
        pcl::PointCloud<Point>::Ptr temp(new pcl::PointCloud<Point>());
        pcl::VoxelGrid<Point> voxel1; // filter to matching-map
        voxel1.setLeafSize(param_planning_voxel_size, param_planning_voxel_size, param_planning_voxel_size);
        voxel1.setInputCloud(in_cloud_xyzp);
        voxel1.filter(*temp);

        // 点云转换
        pcl::transformPointCloud(*temp, *transformed_incloud, t_localizer);

        whole_planning_map += *transformed_incloud;
        if (whole_planning_map.points.size() > param_planning_piece_size) {
            pcl::PointCloud<Point> cloud = whole_planning_map;
            planningMap_queue.push(cloud);
            whole_planning_map.clear();
        }
    }

    // 发布用于生成全局costmap的地图,地面过滤,高度截取到雷达以上0.5m
    if (is_publish_map_for_costmap) {
        filter.setIfClipHeight(true);
        pcl::PointCloud<Point>::Ptr local_no_ground(new pcl::PointCloud<Point>());
        pcl::PointCloud<Point>::Ptr to_globalmap_for_costmap(new pcl::PointCloud<Point>());
        filter.convert(scan_ptr, local_no_ground); // 调用去地面filter导致帧率很慢,TODO::排查

        // 输入点云部分截取
        pcl::PointIndices indices;
        for (size_t i = 0; i < local_no_ground->points.size(); i++) {
            double x = local_no_ground->points[i].x;
            double y = local_no_ground->points[i].y;
            double z = local_no_ground->points[i].z;
            double dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

            if (dist > 1.0 && dist < 30.0)
                indices.indices.push_back(i);
        }
        pcl::ExtractIndices<pcl::PointXYZ> extractor;
        extractor.setInputCloud(local_no_ground);
        extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        extractor.setNegative(false); // false to save the indices
        pcl::PointCloud<Point>::Ptr temp_roi(new pcl::PointCloud<Point>());
        extractor.filter(*temp_roi);

        // 下采样
        pcl::PointCloud<Point>::Ptr temp(new pcl::PointCloud<Point>());
        pcl::VoxelGrid<Point> voxel1; // filter to matching-map
        voxel1.setLeafSize(0.15, 0.15, 0.15);
        voxel1.setInputCloud(temp_roi);
        voxel1.filter(*temp);
        pcl::transformPointCloud(*temp, *to_globalmap_for_costmap, t_localizer);

        whole_cost_map += *to_globalmap_for_costmap;

        if (whole_cost_map.points.size() > param_cost_piece_size) {
            pcl::PointCloud<Point> cloud = whole_cost_map;
            costMap_queue.push(cloud);
            whole_cost_map.clear();
        }
    }

    // 按照步长,处理生成全局匹配用地图等
    double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
    if (shift >= min_add_scan_shift) {
        added_pose.x = current_pose.x;
        added_pose.y = current_pose.y;
        added_pose.z = current_pose.z;
        added_pose.roll = current_pose.roll;
        added_pose.pitch = current_pose.pitch;
        added_pose.yaw = current_pose.yaw;

        pcl::PointCloud<Point>::Ptr to_globalmap(new pcl::PointCloud<Point>()); // piece map也将使用该值
        pcl::PointCloud<Point>::Ptr to_map(new pcl::PointCloud<Point>());
        pcl::VoxelGrid<Point> voxel_grid_filter_tomap; // filter to matching-map
        voxel_grid_filter_tomap.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter_tomap.setInputCloud(transformed_scan_ptr);
        voxel_grid_filter_tomap.filter(*to_map);
        map += *to_map;

        // 发布做下采样后的全局地图, 用于最终ndt匹配定位
        if (is_publish_map_full) {
            // pcl::PointCloud<Point>::Ptr to_globalmap(new pcl::PointCloud<Point>());
            pcl::VoxelGrid<Point> global_voxel_grid_filter_tomap; // filter to global-map(for save)
            voxel_grid_filter_tomap.setLeafSize(param_global_voxel_leafsize, param_global_voxel_leafsize, param_global_voxel_leafsize);
            voxel_grid_filter_tomap.setInputCloud(transformed_scan_ptr);
            voxel_grid_filter_tomap.filter(*to_globalmap);
            global_map += *to_globalmap;
            sensor_msgs::PointCloud2::Ptr msg_globalmap_ptr(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(global_map, *msg_globalmap_ptr);
            refiltered_map_pub.publish(*msg_globalmap_ptr); // topic: "/global/map_full"
        }

        // piece map
        if (param_if_create_piece_map) {
            pcl::PointCloud<Point>::Ptr to_piece_map(new pcl::PointCloud<Point>);
            pcl::VoxelGrid<Point> piece_map_voxeler;
            piece_map_voxeler.setLeafSize(param_piece_map_voxel_size, param_piece_map_voxel_size, param_piece_map_voxel_size);
            piece_map_voxeler.setInputCloud(transformed_scan_ptr);
            piece_map_voxeler.filter(*to_piece_map);

            piece_map_container += *to_piece_map;
            if (piece_map_container.size() > param_piecemap_size) {
                pcl::PointCloud<Point> in_pcd = piece_map_container;
                pieceMap_queue.push(in_pcd);
                piece_map_container.clear();
            }
        }
    } // end if(shift)

    pcl::PointCloud<Point>::Ptr target_map(new pcl::PointCloud<Point>());
    if (shift >= param_min_update_target_map) {
        target_map->points.clear();
        for (auto point : map.points) {
            double len = std::sqrt(std::pow(point.x - current_pose.x, 2) + std::pow(point.y - current_pose.y, 2));
            if (len <= max_scan_range + 2.0) {
                target_map->points.push_back(point);
            }
        }
        // std::cout << target_map->points.size() << std::endl;

        // target_map->header.frame_id = "map";
        // sensor_msgs::PointCloud2::Ptr msg_debug_map_ptr(new sensor_msgs::PointCloud2);
        // pcl::toROSMsg(*target_map, *msg_debug_map_ptr);
        // debug_map_pub.publish(*msg_debug_map_ptr);

        if (_method_type == MethodType::use_pcl)
            pcl_ndt.setInputTarget(map_ptr); // 注意:此时加入的target:map_ptr并不包括刚加入点云的transformed_scan_ptr
        else if (_method_type == MethodType::use_cpu) // 只是说map更新了,因此target也要更新,不要落后太多
        {
            if (_incremental_voxel_update) // TODO:
                cpu_ndt.updateVoxelGrid(transformed_scan_ptr);
            else
                cpu_ndt.setInputTarget(map_ptr);
        } else if (_method_type == MethodType::use_gpu)
#ifdef CUDA_FOUND
            gpu_ndt.setInputTarget(target_map);
#else
            ROS_ERROR("set to ndt_gpu, but no cuda found!");
#endif
        else if (_method_type == MethodType::use_omp)
            omp_ndt.setInputTarget(map_ptr);
    }

    // ################################################################################

    // 整理sensor_msg,发布更新后的(且降采样后的)全局地图,以及位置
    // start5
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    matching_map_pub.publish(*map_msg_ptr); // "map_for_localmatch" 每一帧都发布map(全局map)

    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = current_scan_time;
    current_pose_msg.pose.position.x = current_pose.x;
    current_pose_msg.pose.position.y = current_pose.y;
    current_pose_msg.pose.position.z = current_pose.z;
    current_pose_msg.pose.orientation.x = q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();

    current_pose_pub.publish(current_pose_msg); // TODO:每一帧都发布current_pose
    ros::Time test_time_6 = ros::Time::now(); // TODO:
    // end5

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
        std::cout << "shift: " << shift << std::endl;

        std::cout << "---------------------------------------------" << std::endl;
    }
}

} // namespace FAST_NDT
