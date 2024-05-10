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

        // 将接收到的点云添加到累积点云中
        *accumulated_cloud += input_cloud;

        // 检查累积帧数是否达到30帧
        if (++frame_count >= 30)
        {
            ROS_INFO("Accumulated 10 frames of point clouds. Saving to PCD file.");

            // 移除0,0,0处的点
            accumulated_cloud->erase(
                std::remove_if(accumulated_cloud->points.begin(), accumulated_cloud->points.end(),
                               [](const pcl::PointXYZ &p) {
                                   return (p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
                               }),
                accumulated_cloud->points.end());

            // 保存累积点云为pcd文件
            pcl::io::savePCDFileASCII("/home/yuezk/catkin_ws_03/src/pointCloudtoPointCloud2/src/accumulated_cloud.pcd", *accumulated_cloud);

            ROS_INFO("PCD file saved. Shutting down the node.");

            // 关闭节点
            ros::shutdown();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
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
