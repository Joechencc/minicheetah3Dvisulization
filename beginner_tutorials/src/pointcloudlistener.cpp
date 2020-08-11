#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include "pcl/pcl_type.hpp"
#include <pcl/filters/passthrough.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;

int count = 0;
//int row = 141; // 10 to 150 cm, 1cm away
//int column = 161; //-80 cm to +80, 1cm away

//double pc_matrix[141][201];

void callback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >& msg)
//void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{	
	///////lcm 
	lcm::LCM *lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=255");

	if(!lcm->good()){
	  return;
	}
	
	pcl::pcl_type mydata;

	for (int i=0; i < 30; i++){
		mydata.xarray[i] = 0;
		mydata.yarray[i] = 0;
		mydata.zarray[i] = 0;
	}

	////////downsampling
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInput;
    	cloudInput.reset(new pcl::PointCloud<pcl::PointXYZ> (*msg));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>); 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered2 (new pcl::PointCloud<pcl::PointXYZ>);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloudInput);
	sor.setLeafSize (0.5f, 0.5f, 0.5f );
	sor.filter (*cloudFiltered); 
  	ros::Rate loop_rate(10);

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloudFiltered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.10,1.5);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.8,0.8);
	pass.filter(*cloudFiltered2);
	//pcl::PassThrough<PointXYZ> filter;
	//filter.setFilterFieldName("x");

        ros::NodeHandle n;
        ros::Publisher pc_pub = n.advertise<pcl::PointCloud<pcl::PointNormal> >("normal_pointcloud", 1);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normalEstimation;

       // The normal estimation object will use it to find nearest neighbors.
 	typename pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//kdtree->setInputCloud(msg);
	normalEstimation.setSearchMethod(kdtree); 
  	normalEstimation.setInputCloud (cloudFiltered2);
	normalEstimation.setRadiusSearch(0.3);  
	float nx,ny,nz,kurvature;
	pcl::PointCloud<pcl::PointNormal> cloud_normals_;

	normalEstimation.setViewPoint(0, 0, 1.0);																																															
	normalEstimation.compute (cloud_normals_); 
	int index = 0;
	int index2 = 0;

	int temp_size = cloudFiltered2->points.size();
	int limit_size = 30;
	if (temp_size > limit_size){
		temp_size = 30;
	}
	//ROS_INFO("point:%ld",temp_size);

	//double x_limit_lower = -1.5;
	//double x_limit_upper = 1.5;
	//double z_limit_upper = 3;
	//memset(pc_matrix, 0, sizeof(pc_matrix[0][0]) * row * column);
	//std::ofstream myfile;
  	//myfile.open("/home/chaonew/Desktop/temp_folder/example.txt");
	for(unsigned int i=0; i < temp_size ; i=i+1) {
	// x range -35m to 35 m, y ranges -25 to 25m, z ranges 0 to 66
	    cloud_normals_.points[index].x = cloudFiltered2->points[i].x;
	    cloud_normals_.points[index].y = cloudFiltered2->points[i].y;
	    cloud_normals_.points[index].z = cloudFiltered2->points[i].z;
	    //int index_z = (cloud_normals_.points[index].z / 0.01) - 15;
	    //int index_x = (cloud_normals_.points[index].x / 0.01) + 250;	

	    //pc_matrix[index_z][index_x]= -cloud_normals_.points[index].y; //y axis is pointing downward
	    //ROS_INFO("index2:%d",index2);		
	    if(index2!=30){	
		mydata.xarray[index2] =cloud_normals_.points[index].x;
		mydata.yarray[index2] =cloud_normals_.points[index].y;
		mydata.zarray[index2] =cloud_normals_.points[index].z;
		//ROS_INFO("mydata.xarray[index2]:%lf",mydata.xarray[index2]);
	    }
	    else if((index2==30) &&(mydata.zarray[0]>cloud_normals_.points[index].z)){
		mydata.xarray[0] =cloud_normals_.points[index].x;
		mydata.yarray[0] =cloud_normals_.points[index].y;	
		mydata.zarray[0] =cloud_normals_.points[index].z;
	    }		
	    index2 ++;
	    if(index2>30){
		index2 = 30;
	    }
	    index ++;
	}
	//for(int i=10; i< row; i=i+10){
		//for(int j=10; j< column; j=j+10){
		//	if (j!=500){
		//		myfile << pc_matrix[i][j] << ", ";
		//	}
		//	else{
		//		myfile << pc_matrix[i][j] << '\n';
		//	}
		//}
	//}
	//myfile.close();
	lcm->publish("pcl_topic", &mydata);	
	pc_pub.publish(cloud_normals_);
	ros::spinOnce();

    	loop_rate.sleep();


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<PointCloud>("/d400/depth/color/points", 1, callback);
    	ros::Rate loop_rate(30);

	while (ros::ok()) {
        	//update joint_state
        	ros::spinOnce();
        	loop_rate.sleep();

    	}
}
