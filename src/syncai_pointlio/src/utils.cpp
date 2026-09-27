#include "syncai_pointlio/utils.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pc2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    const size_t point_num = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    if (filter_num < 1)
        filter_num = 1;
    cloud->reserve(point_num / filter_num + 1);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    // intensity is optional (Isaac Sim clouds often carry x/y/z only)
    bool has_intensity = false;
    for (const auto &f : msg->fields)
        if (f.name == "intensity") { has_intensity = true; break; }
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_i;
    if (has_intensity)
        it_i.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "intensity"));

    const double min_sq = min_range * min_range;
    const double max_sq = max_range * max_range;
    size_t idx = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++idx)
    {
        float intensity = 0.0f;
        if (has_intensity) { intensity = *(*it_i); ++(*it_i); }
        if (idx % filter_num != 0)
            continue;
        float x = *it_x, y = *it_y, z = *it_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;
        double r2 = static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z;
        if (r2 < min_sq || r2 > max_sq)
            continue;
        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity;
        p.curvature = 0.0f; // no per-point time -> snapshot, no deskew
        cloud->push_back(p);
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
