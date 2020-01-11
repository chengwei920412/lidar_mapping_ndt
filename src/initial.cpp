#include <ndt_mapping.h>

namespace ndt_mapping {

LidarMapping::LidarMapping()
{
    _method_type = MethodType::use_cpu;

    // 定义各种差异值(两次采集数据之间的差异,包括点云位置差异,imu差异,odom差异,imu-odom差异)
    diff = 0.0;
    diff_x = diff_y = diff_z = 0.0; // current_pose - previous_pose // 定义两帧点云差异值 --以确定是否更新点云等

    // 定义速度值 --包括实际速度值,和imu取到的速度值
    current_velocity_x = 0.0;
    current_velocity_y = 0.0;
    current_velocity_z = 0.0;

    current_velocity_imu_x = 0.0;
    current_velocity_imu_y = 0.0;
    current_velocity_imu_z = 0.0;

    Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity(); // 保存GPS信号的变量

    _use_imu = false;
    _use_odom = false;
    _imu_upside_down = false; // 用以解决坐标系方向(正负变换)问题 (比如x变更为-x等)

    _incremental_voxel_update = false; // ??????????

    // is_first_piece_map = true;

    t_localizer = Eigen::Matrix4f::Identity();
    t_base_link = Eigen::Matrix4f::Identity();

    initial_scan_loaded = false; // 用以确定是否为第一帧点云(第一帧点云不做匹配,直接添加到地图中去)
}

void LidarMapping::param_initial(ros::NodeHandle& nh, ros::NodeHandle& privateHandle)
{
    std::cout << "******************************************" << std::endl;
    std::cout << ">> NDT_param_initial <<" << std::endl;
    privateHandle.param<std::string>("base_frame", param_base_frame, "");
    privateHandle.param<std::string>("laser_frame", param_laser_frame, "");
    if (param_base_frame == "" || param_laser_frame == "") {
        std::cout << "base_frame: " << param_base_frame << std::endl;
        std::cout << "laser_frame:  " << param_laser_frame << std::endl;
        ROS_ERROR("param: base_link & laser_link unset !");
        return;
    }

    privateHandle.param<double>("lidar_height", lidar_height, 1.7);
    filter.setLidarHeight(lidar_height);
    param = new Param();

    privateHandle.param<double>("tf_timeout", param_tf_timeout, 0.05);
    privateHandle.param<bool>("visualize", param_visualize, false);

    privateHandle.param<float>("ndt_resolution", ndt_res, 1.0);
    privateHandle.param<double>("ndt_step_size", step_size, 0.1);
    privateHandle.param<double>("ndt_trans_eps", trans_eps, 0.01);
    privateHandle.param<int>("ndt_max_iter", max_iter, 100);
    privateHandle.param<double>("voxel_leaf_size", voxel_leaf_size, 0.5);
    privateHandle.param<double>("min_scan_range", min_scan_range, 0.5);
    privateHandle.param<double>("max_scan_range", max_scan_range, 100);
    privateHandle.param<double>("min_add_scan_shift", min_add_scan_shift, 200.0);

    privateHandle.param<double>("global_voxel_leafsize", param_global_voxel_leafsize, 1.0);

    privateHandle.param<double>("min_update_target_map", param_min_update_target_map, 0.);
    privateHandle.param<double>("extract_length", param_extract_length, 0.);
    privateHandle.param<double>("extract_width", param_extract_width, 0.);

    std::cout << "min_update_target_map:" << param_min_update_target_map << std::endl;
    std::cout << "extract_length: " << param_extract_length << std::endl;
    std::cout << "extract_width: " << param_extract_width << std::endl;
    std::cout << std::endl;
    if (param_min_update_target_map == 0. || param_extract_length == 0. || param_extract_width == 0.) {
        ROS_ERROR("min_update_target extract_length and width unset !");
        return;
    }
    std::cout << "ndt_res: " << ndt_res << std::endl;
    std::cout << "step_size: " << step_size << std::endl;
    std::cout << "trans_epsilon: " << trans_eps << std::endl;
    std::cout << "max_iter: " << max_iter << std::endl;
    std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
    std::cout << "min_scan_range: " << min_scan_range << std::endl;
    std::cout << "max_scan_range: " << max_scan_range << std::endl;
    std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
    std::cout << std::endl;

    privateHandle.param<bool>("use_imu", _use_imu, false);
    privateHandle.param<bool>("use_odom", _use_odom, false);
    privateHandle.param<bool>("imu_upside_down", _imu_upside_down, false);

    std::cout << "use imu: " << _use_imu << std::endl;
    std::cout << "use_odom: " << _use_odom << std::endl;
    std::cout << "reverse imu: " << _imu_upside_down << std::endl;
    std::cout << std::endl;

    privateHandle.param<std::string>("imu_topic", _imu_topic, "/imu_raw");
    privateHandle.param<std::string>("odom_topic", _odom_topic, "/odom_raw");
    privateHandle.param<std::string>("lidar_topic", _lidar_topic, "/velodyne_points");

    std::cout << "imu topic: " << _imu_topic << std::endl;
    std::cout << "odom topic: " << _odom_topic << std::endl;
    std::cout << "lidar topic: " << _lidar_topic << std::endl;
    std::cout << std::endl;

    privateHandle.param<bool>("incremental_voxel_update", _incremental_voxel_update, true);
    std::cout << "incremental_voxel_update: " << _incremental_voxel_update << std::endl; // 是否在cpu_ndt里做update????

    // piece map params
    privateHandle.param<bool>("if_create_piece_map", param_if_create_piece_map, false);
    privateHandle.param<std::string>("piece_map_folder", param_piece_map_folder, "undefined");

    privateHandle.param<int>("piecemap_size", param_piecemap_size, 20000);
    privateHandle.param<double>("piece_map_voxel_size", param_piece_map_voxel_size, 0.5);

    param->is_create_pieceMap = param_if_create_piece_map;
    param->pieceMap_folder = param_piece_map_folder;

    // planning map params
    privateHandle.param<bool>("is_publish_map_for_planning", is_publish_map_for_planning, "false");
    privateHandle.param<std::string>("planning_map_folder", param_planning_map_folder, "None");
    privateHandle.param<int>("planning_piece_size", param_planning_piece_size, 50000);
    privateHandle.param<double>("planning_voxel_size", param_planning_voxel_size, 0.5);
    if (is_publish_map_for_planning) {
        std::cout << "Will publish whole map (default filter with 0.1) to " << param_planning_map_folder << std::endl;
        param->is_create_planningMap = is_publish_map_for_planning;
        param->planningMap_folder = param_planning_map_folder;
    }

    // cost map params
    privateHandle.param<bool>("is_publish_map_for_costmap", is_publish_map_for_costmap, false);
    privateHandle.param<std::string>("cost_map_folder", param_cost_map_folder, "None");
    privateHandle.param<int>("cost_piece_size", param_cost_piece_size, 20000);
    privateHandle.param<double>("costmap_voxel_size", param_costmap_voxel_size, 0.5);

    if (is_publish_map_for_costmap) {
        std::cout << "Will publish map for costmap to " << param_cost_map_folder << std::endl;
        param->is_create_costMap = is_publish_map_for_costmap;
        param->costMap_folder = param_cost_map_folder;
    }

    privateHandle.param<bool>("is_publish_map_full", is_publish_map_full, false);
    if (is_publish_map_full) {
        std::cout << "Will publish full map." << std::endl;
    }

    //		!! 注意自定义类型格式在获取参数时的实现方式 !!
    int method_type_temp;
    privateHandle.param<int>("ndt_method_type", method_type_temp, 1);
    _method_type = static_cast<MethodType>(method_type_temp);

    switch (method_type_temp) {
    case 0:
        std::cout << ">> Use PCL NDT <<" << std::endl;
        break;
    case 1:
        std::cout << ">> Use CPU NDT <<" << std::endl;
        break;
    case 2:
        std::cout << ">> Use GPU NDT <<" << std::endl;
        break;
    case 3:
        std::cout << ">> Use OMP NDT <<" << std::endl;
        break;
    default:
        ROS_ERROR("Invalid method type of NDT");
        exit(1);
    }
    std::cout << std::endl;

    double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
    privateHandle.param<double>("tf_x", _tf_x, 0.);
    privateHandle.param<double>("tf_y", _tf_y, 0.);
    privateHandle.param<double>("tf_z", _tf_z, 0.);
    privateHandle.param<double>("tf_roll", _tf_roll, 0.);
    privateHandle.param<double>("tf_pitch", _tf_pitch, 0.);
    privateHandle.param<double>("tf_yaw", _tf_yaw, 0.);

    Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);

    Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
    tf_base_laser = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    tf_laser_base = tf_base_laser.inverse();

    previous_pose.x = 0.0;
    previous_pose.y = 0.0;
    previous_pose.z = 0.0;
    previous_pose.roll = 0.0;
    previous_pose.pitch = 0.0;
    previous_pose.yaw = 0.0;

    ndt_pose.x = 0.0;
    ndt_pose.y = 0.0;
    ndt_pose.z = 0.0;
    ndt_pose.roll = 0.0;
    ndt_pose.pitch = 0.0;
    ndt_pose.yaw = 0.0;

    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.z = 0.0;
    current_pose.roll = 0.0;
    current_pose.pitch = 0.0;
    current_pose.yaw = 0.0;

    guess_pose.x = 0.0;
    guess_pose.y = 0.0;
    guess_pose.z = 0.0;
    guess_pose.roll = 0.0;
    guess_pose.pitch = 0.0;
    guess_pose.yaw = 0.0;

    added_pose.x = 0.0;
    added_pose.y = 0.0;
    added_pose.z = 0.0;
    added_pose.roll = 0.0;
    added_pose.pitch = 0.0;
    added_pose.yaw = 0.0;

    diff_x = 0.0;
    diff_y = 0.0;
    diff_z = 0.0;
    diff_yaw = 0.0;

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

    if (_method_type == MethodType::use_pcl) {
        pcl_ndt.setTransformationEpsilon(trans_eps);
        pcl_ndt.setStepSize(step_size);
        pcl_ndt.setResolution(ndt_res);
        pcl_ndt.setMaximumIterations(max_iter);
        // pcl_ndt.setInputSource(filtered_scan_ptr);
    } else if (_method_type == MethodType::use_cpu) {
        cpu_ndt.setTransformationEpsilon(trans_eps);
        cpu_ndt.setStepSize(step_size);
        cpu_ndt.setResolution(ndt_res);
        cpu_ndt.setMaximumIterations(max_iter);
        // cpu_ndt.setInputSource(filtered_scan_ptr);
    } else if (_method_type == MethodType::use_gpu) {
#ifdef CUDA_FOUND
        gpu_ndt.setTransformationEpsilon(trans_eps);
        gpu_ndt.setStepSize(step_size);
        gpu_ndt.setResolution(ndt_res);
        gpu_ndt.setMaximumIterations(max_iter);
        // gpu_ndt.setInputSource(filtered_scan_ptr);
#else
        ROS_ERROR("set to use ndt_gpu, but no cuda found!");
#endif
    } else if (_method_type == MethodType::use_omp) {
        omp_ndt.setTransformationEpsilon(trans_eps);
        omp_ndt.setStepSize(step_size);
        omp_ndt.setResolution(ndt_res);
        omp_ndt.setMaximumIterations(max_iter);
        // omp_ndt.setInputSource(filtered_scan_ptr);
    } else {
        ROS_ERROR("Please Define _method_type to conduct NDT");
        exit(1);
    }
}
}