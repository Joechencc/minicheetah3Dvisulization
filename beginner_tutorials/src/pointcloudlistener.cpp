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


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;

int count = 0;
int row = 501; //-250 cm to +250, 1cm away
int column = 501; // 20 to 520 cm, 2cm away

double pc_matrix[501][501];

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
//void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{	
	////////downsampling
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInput;
    	cloudInput.reset(new pcl::PointCloud<pcl::PointXYZ> (*msg));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloudInput);
	sor.setLeafSize (0.15f, 0.01f, 0.15f );
	sor.filter (*cloudFiltered); 
  	ros::Rate loop_rate(10);
	//pcl::PassThrough<PointXYZ> filter;
	//filter.setFilterFieldName("x");

        ros::NodeHandle n;
        ros::Publisher pc_pub = n.advertise<pcl::PointCloud<pcl::PointNormal> >("normal_pointcloud", 1);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normalEstimation;

       // The normal estimation object will use it to find nearest neighbors.
 	typename pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//kdtree->setInputCloud(msg);
	normalEstimation.setSearchMethod(kdtree); 
  	normalEstimation.setInputCloud (cloudFiltered);
	normalEstimation.setRadiusSearch(0.3);  
	float nx,ny,nz,kurvature;
	pcl::PointCloud<pcl::PointNormal> cloud_normals_;

	normalEstimation.setViewPoint(0, 0, 1.0);																																															
	normalEstimation.compute (cloud_normals_); 
	int index = 0;
	long temp_size = cloudFiltered->points.size();
	ROS_INFO("point:%ld",temp_size);

	double x_limit_lower = -1.5;
	double x_limit_upper = 1.5;
	double z_limit_upper = 3;

	memset(pc_matrix, 0, sizeof(pc_matrix[0][0]) * row * column);

	std::ofstream myfile;
	//std::string file_name=;
  	myfile.open("/home/chaonew/Desktop/temp_folder/example.txt");
	for(unsigned int i=0; i < temp_size; i=i+1) {
	// x range -35m to 35 m, y ranges -25 to 25m, z ranges 0 to 66
	    cloud_normals_.points[index].x = cloudFiltered->points[i].x;
	    cloud_normals_.points[index].y = cloudFiltered->points[i].y;
	    cloud_normals_.points[index].z = cloudFiltered->points[i].z;
	    int index_z = (cloud_normals_.points[index].z / 0.01) - 20;
	    int index_x = (cloud_normals_.points[index].x / 0.01) + 250;	

	    
	    if ((index_x < row) && (index_z < column)){
  		//ROS_INFO("x:%d",index_x);
	    	//ROS_INFO("z:%d",index_z);
		pc_matrix[index_z][index_x]= -cloud_normals_.points[index].y; //y axis is pointing downward
		//ROS_INFO("y:%f",pc_matrix[index_z][index_x]);
		//ROS_INFO("pc_matrix_x:%f",cloud_normals_.points[index].x);
		//ROS_INFO("pc_matrix_z:%f",cloud_normals_.points[index].z);
		//ROS_INFO("pc_matrix_y:%f",pc_matrix[index_z][index_x]);
	    }
	    index ++;
	    //ROS_INFO("point:%d",index);
	}
	for(int i=10; i< row; i=i+10){
		for(int j=10; j< column; j=j+10){
			if (j!=500){
				myfile << pc_matrix[i][j] << ", ";
			}
			else{
				myfile << pc_matrix[i][j] << '\n';
			}
		}
	}
	myfile.close();
	
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
