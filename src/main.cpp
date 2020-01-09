#include <ndt_mapping.h>

using namespace ndt_mapping;

void LidarMapping::run(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{

    param_initial(nh, private_nh);

    map.header.frame_id = "map";
    global_map.header.frame_id = "map";
    global_map_no_ground.header.frame_id = "map";
    whole_planning_map.header.frame_id = "map";
    whole_cost_map.header.frame_id = "map";

    debug_map_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/deug_map", 1000);
    matching_map_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/globalmap/map_for_localmatch", 1000);
    refiltered_map_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/globalmap/map_full", 1000);
    current_pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

    pthread_create(&thread, NULL, thread_map_saver, param);

    points_sub = private_nh.subscribe(_lidar_topic, 1000, &LidarMapping::points_callback, this);
    odom_sub = private_nh.subscribe(_odom_topic, 1000, &LidarMapping::odom_callback, this);

    ros::Rate(10);
    ros::spin();

    pthread_join(thread, NULL);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_mapping_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ndt_mapping::LidarMapping mapping;
    mapping.run(nh, private_nh);
    return 0;
}
