#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class PointCloudAccumulator
{
public:
    PointCloudAccumulator()
    {
        // 订阅点云话题
        sub = nh.subscribe("/scan/livox", 1, &PointCloudAccumulator::pointCloudCallback, this);
        pub = nh.advertise<sensor_msgs::PointCloud2>("/scan/livox2",10);
        // 初始化累积点云
        accumulated_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    void pointCloudCallback(const sensor_msgs::PointCloudPtr &cloud_msg)
    {
        ROS_INFO("Received a point cloud message.");

        sensor_msgs::PointCloud2 cloud2_msg;
        sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, cloud2_msg);

        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg(cloud2_msg, input_cloud);
        pub.publish(cloud2_msg);

    }

private:
    ros::NodeHandle nh;

    ros::Subscriber sub;
    ros::Publisher pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
    int frame_count = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_accumulator_node");
    PointCloudAccumulator point_cloud_accumulator;

    ROS_INFO("PointCloud Accumulator Node is running.");

    ros::spin();

    return 0;
}
