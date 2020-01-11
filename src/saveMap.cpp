#include <ndt_mapping.h>

namespace ndt_mapping {

std::queue<pcl::PointCloud<Point>> LidarMapping::pieceMap_queue = std::queue<pcl::PointCloud<Point>>();
std::queue<pcl::PointCloud<Point>> LidarMapping::planningMap_queue = std::queue<pcl::PointCloud<Point>>();
std::queue<pcl::PointCloud<Point>> LidarMapping::costMap_queue = std::queue<pcl::PointCloud<Point>>();

void* LidarMapping::thread_map_saver(void* params)
{
    Param* param = (Param*)params;

    if (param->is_create_pieceMap && param->pieceMap_folder[param->pieceMap_folder.length() - 1] != '/')
        param->pieceMap_folder += "/";
    if (param->is_create_planningMap && param->planningMap_folder[param->pieceMap_folder.length() - 1] != '/')
        param->planningMap_folder += "/";
    if (param->is_create_costMap && param->costMap_folder[param->pieceMap_folder.length() - 1] != '/')
        param->costMap_folder += "/";

    ros::NodeHandle nh_, pnh_("~");
    ros::Rate loop_rate(1);
    int cnt_pieceMap = 0, cnt_costMap = 0, cnt_planningMap = 0;

    while (ros::ok()) {
        if (!pieceMap_queue.empty()) {
            pcl::PointCloud<Point> cloud = pieceMap_queue.front();
            pieceMap_queue.pop();
            std::stringstream pieceMap_name;
            pieceMap_name << param->pieceMap_folder << cnt_pieceMap << "_pieceMap.pcd";
            cnt_pieceMap++;
            pcl::io::savePCDFile<Point>(pieceMap_name.str(), cloud);
            continue;
        }
        if (!planningMap_queue.empty()) {
            pcl::PointCloud<Point> cloud = planningMap_queue.front();
            planningMap_queue.pop();
            std::stringstream planningMap_name;
            planningMap_name << param->planningMap_folder << cnt_planningMap << "_pieceMap.pcd";
            cnt_planningMap++;
            pcl::io::savePCDFile<Point>(planningMap_name.str(), cloud);
            continue;
        }
        if (!costMap_queue.empty()) {
            pcl::PointCloud<Point> cloud = costMap_queue.front();
            costMap_queue.pop();
            std::stringstream costMap_name;
            costMap_name << param->costMap_folder << cnt_costMap << "_pieceMap.pcd";
            cnt_costMap++;
            pcl::io::savePCDFile<Point>(costMap_name.str(), cloud);
            continue;
        }
        loop_rate.sleep();
    }

    return nullptr;
}

void LidarMapping::saveMapForPlanning(const pcl::PointCloud<PointI>::Ptr& input)
{
    pcl::PointCloud<PointI>::Ptr clipped_cloud(new pcl::PointCloud<PointI>());
    pcl::PointCloud<Point>::Ptr sampled_cloud(new pcl::PointCloud<Point>());
    pcl::PointCloud<PointI>::Ptr transformed_incloud(new pcl::PointCloud<PointI>());

    clipCloudbyMinAndMaxDisAndHeight(input, clipped_cloud, 1.0, 50.0, -1.3);

    downSampleFilter(clipped_cloud, sampled_cloud, param_planning_voxel_size);

    pcl::transformPointCloud(*sampled_cloud, *transformed_incloud, t_localizer);

    whole_planning_map += *transformed_incloud;

    if (int(whole_planning_map.points.size()) > param_planning_piece_size) {
        pcl::PointCloud<Point> cloud = whole_planning_map;
        planningMap_queue.push(cloud);
        whole_planning_map.clear();
    }
}

void LidarMapping::saveMapForCostmap(const pcl::PointCloud<PointI>::Ptr& input)
{
    pcl::PointCloud<Point>::Ptr local_no_ground(new pcl::PointCloud<Point>());
    pcl::PointCloud<Point>::Ptr to_globalmap_for_costmap(new pcl::PointCloud<Point>());
    pcl::PointCloud<Point>::Ptr clipped_cloud(new pcl::PointCloud<Point>());
    pcl::PointCloud<Point>::Ptr sampled_cloud(new pcl::PointCloud<Point>());

    // 去地面
    filter.setIfClipHeight(true);
    filter.convert(input, local_no_ground); // 调用去地面filter导致帧率很慢,TODO::排查

    // 截取点云
    clipCloudbyMinAndMaxDisAndHeight(local_no_ground, clipped_cloud, 1.0, 30.0, 0.3);

    // 下采样
    downSampleFilter(clipped_cloud, sampled_cloud, 0.15);

    // 转换点云
    pcl::transformPointCloud(*sampled_cloud, *to_globalmap_for_costmap, t_localizer);

    whole_cost_map += *to_globalmap_for_costmap;

    if (int(whole_cost_map.points.size()) > param_cost_piece_size) {
        pcl::PointCloud<Point> cloud = whole_cost_map;
        costMap_queue.push(cloud);
        whole_cost_map.clear();
    }
}

void LidarMapping::savePieceMap(const pcl::PointCloud<PointI>::Ptr& input)
{
    pcl::PointCloud<Point>::Ptr to_piece_map(new pcl::PointCloud<Point>);
    downSampleFilter(input, to_piece_map, param_piece_map_voxel_size);

    piece_map_container += *to_piece_map;
    if (int(piece_map_container.size()) > param_piecemap_size) {
        pcl::PointCloud<Point> in_pcd = piece_map_container;
        pieceMap_queue.push(in_pcd);
        piece_map_container.clear();
    }
}
}