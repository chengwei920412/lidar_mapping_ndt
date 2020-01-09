#include <ndt_mapping.h>

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

    // #pragma omp for
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
}