
#include <ndt_mapping.h>
#include <pcl/common/transforms.h> //allows us to use pcl::transformPointCloud function
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace ndt_mapping {
double LidarMapping::warpToPm(double a_num, const double a_max)
{
    if (a_num >= a_max) {
        a_num -= 2.0 * a_max;
    }
    return a_num;
}

double LidarMapping::warpToPmPi(double a_angle_rad)
{
    return warpToPm(a_angle_rad, M_PI);
}

double LidarMapping::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}

void LidarMapping::clipCloudbyMinAndMaxDis(const pcl::PointCloud<Point>::Ptr& input_cloud, pcl::PointCloud<Point>::Ptr& output_cloud, const double& min_dis, const double& max_dis)
{
    pcl::ExtractIndices<Point> cliper;

    cliper.setInputCloud(input_cloud);

    pcl::PointIndices indices;

#pragma omp for
    for (size_t i = 0; i < input_cloud->points.size(); i++) {
        Point point = input_cloud->points[i];
        double r = std::sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
        if (min_dis < r && r < max_dis) {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false); //ture to remove the indices
    cliper.filter(*output_cloud);
}

void LidarMapping::clipCloudbyMinAndMaxDisAndHeight(const pcl::PointCloud<Point>::Ptr& input_cloud, pcl::PointCloud<Point>::Ptr& output_cloud, const double& min_dis, const double& max_dis, const double& height)
{

    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < input_cloud->points.size(); i++) {
        double x = input_cloud->points[i].x;
        double y = input_cloud->points[i].y;
        double z = input_cloud->points[i].z;
        double dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

        if (dist > min_dis && dist < max_dis && z < height)
            indices.indices.push_back(i);
    }
    pcl::ExtractIndices<PointI> cliper;
    cliper.setInputCloud(input_cloud);
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false); // false to save the indices
    cliper.filter(*output_cloud);
}

void LidarMapping::downSampleFilter(const pcl::PointCloud<Point>::Ptr& input_cloud, pcl::PointCloud<Point>::Ptr& output_cloud, const double& voxel_size)
{
    // 下采样
    pcl::PointCloud<Point>::Ptr temp(new pcl::PointCloud<Point>());
    pcl::VoxelGrid<Point> voxel1;
    voxel1.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel1.setInputCloud(input_cloud);
    voxel1.filter(*output_cloud);
}

void visualPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& first_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& second_cloud)
{
    // Visualization
    printf("\nPoint cloud colors :  white  = original point cloud \n"
           "                        red  = transformed point cloud\n");

    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(first_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud(first_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(second_cloud, 230, 20, 20); // Red
    viewer.addPointCloud(second_cloud, transformed_cloud_color_handler, "transformed_cloud");

    //Adds 3D axes describing a coordinate system to screen at 0,0,0.
    viewer.addCoordinateSystem(1.0, 0, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }
}
}