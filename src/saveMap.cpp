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
}