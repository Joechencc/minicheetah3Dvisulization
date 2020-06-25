#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
 
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
 
void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
	for (size_t l = 0; l < msg->points.size(); l++){
		ROS_INFO("X:%f", msg->points[l].x);
		ROS_INFO("Y:%f", msg->points[l].y);
		ROS_INFO("Z:%f", msg->points[l].z);
		ROS_INFO("R:%d", msg->points[l].r);
		ROS_INFO("G:%d", msg->points[l].g);
		ROS_INFO("B:%d", msg->points[l].b);
		//uint32_t rgb = *reinterpret_cast<int*>(&msg->points[l].rgb);
		//ROS_INFO("R:%d", (rgb >> 16) & 0x0000ff);
		//ROS_INFO("G:%d", (rgb >> 8)  & 0x0000ff);
		//ROS_INFO("B:%d", (rgb)       & 0x0000ff);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, callback);
	ros::spin();
}
