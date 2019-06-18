#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <sensor_msgs/PointCloud2.h>

static const std::string INPUT_TOPIC = "/carla/ego_vehicle/lidar/lidar1/point_cloud";
static const std::string PUBLISH_TOPIC = "/filtered_output";

ros::Publisher pub;


void lidar_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //containers
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_voxel(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_plane (new pcl::PCLPointCloud2);

    //ros -> pcl
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    //voxel grid filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.25, 0.25, 0.25);
    sor.filter (*cloud_voxel);

    //pass through filter
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloud_voxel);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2, 1);
    pass.filter(*cloud_plane);

    //pcl -> ros
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_plane, output);

    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_test");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(INPUT_TOPIC, 1000, lidar_cb);
    pub = n.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

    ros::spin(); 

    return 0;
}