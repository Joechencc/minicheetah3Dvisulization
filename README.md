# minicheetah3Dvisulization
This will combine with MiniCheetah code and doing visualization



Installation Guide:
1. PCL 1.8 for Kinetic   
https://gist.github.com/IgniparousTempest/ce5fadbe742526d10d6bdbf15c3a3fe7

    https://github.com/PointCloudLibrary/pcl.git       git checkout tags/pcl-1.8.0 -b pcl-1.8.0

    1.11 for melodic          git checkout tags/pcl-1.11.0 -b pcl-1.11.0
    
2. use beginner_tutorial for listening PCl
3. Vision_opencv  https://github.com/ros-perception/vision_opencv  (Branch:need to be checked out base on your ROS version)
4. Realsense_ROS https://github.com/IntelRealSense/realsense-ros
    Realsense https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
5. depthimage_to_laserscan https://github.com/ros-perception/depthimage_to_laserscan
6. rtabmap https://github.com/introlab/rtabmap_ros refer to this
7. lcm-to-ROS 
8. new_joint_state(you may buld this at last)

Replace all launch file in realsense2_camera, and rtabmap.launch in package rtabmap_ros.

If you have any question, please contact joecc@umich.edu


![](Resource/cheetah.jpg)
![](Resource/3D1.PNG)
![](Resource/raw_pointcloud.PNG)
![](Resource/3DVisu.gif)
![](Resource/joint.gif)


