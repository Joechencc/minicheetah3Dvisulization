#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "new_joint_pub/leg_control_data_lcmt.h"
// #include "new_joint_pub/leg_control_data_lcmt.hpp"
// #include "std_msgs/String.h"


// #include <tf/transform_broadcaster.h>


void legCallback(const new_joint_pub::leg_control_data_lcmt::ConstPtr &msg)
{
        sensor_msgs::JointState joint_state;
        ros::NodeHandle n;
        ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
        // ros::Rate loop_rate(30);

        joint_state.header.stamp = ros::Time::now();
        // joint_state.name.resize(3);
        // joint_state.position.resize(3);
        // joint_state.name[0] ="swivel";
        // joint_state.position[0] = swivel;
        // joint_state.name[1] ="tilt";
        // joint_state.position[1] = tilt;
        // joint_state.name[2] ="periscope";
        // joint_state.position[2] = height;

        joint_state.name.resize(12);
        joint_state.position.resize(12);
        joint_state.name[0] ="torso_to_abduct_fl_j";
        joint_state.position[0] = msg->q[3];
        joint_state.name[1] ="abduct_fl_to_thigh_fl_j";
        joint_state.position[1] = msg->q[4];
        joint_state.name[2] ="thigh_fl_to_knee_fl_j";
        joint_state.position[2] = msg->q[5];
        joint_state.name[3] ="torso_to_abduct_fr_j";
        joint_state.position[3] = msg->q[0];
        joint_state.name[4] ="abduct_fr_to_thigh_fr_j";
        joint_state.position[4] = msg->q[1];
        joint_state.name[5] ="thigh_fr_to_knee_fr_j";
        joint_state.position[5] = msg->q[2];
        joint_state.name[6] ="torso_to_abduct_hl_j";
        joint_state.position[6] = msg->q[9];
        joint_state.name[7] ="abduct_hl_to_thigh_hl_j";
        joint_state.position[7] = msg->q[10];
        joint_state.name[8] ="thigh_hl_to_knee_hl_j";
        joint_state.position[8] = msg->q[11];
        joint_state.name[9] ="torso_to_abduct_hr_j";
        joint_state.position[9] = msg->q[6];
        joint_state.name[10] ="abduct_hr_to_thigh_hr_j";
        joint_state.position[10] = msg->q[7];
        joint_state.name[11] ="thigh_hr_to_knee_hr_j";
        joint_state.position[11] = msg->q[8];

        // update transform
        // (moving in a circle with radius=2)

        // odom_trans.header.stamp = ros::Time::now();
        // odom_trans.transform.translation.x = cos(angle)*2;
        // odom_trans.transform.translation.y = sin(angle)*2;
        // odom_trans.transform.translation.z = .7;
        // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        // broadcaster.sendTransform(odom_trans);

        // Create new robot state

        // tilt += tinc;
        // if (tilt<-.5 || tilt>0) tinc *= -1;
        // height += hinc;
        // if (height>.2 || height<0) hinc *= -1;
        // swivel += degree;
        // angle += degree/4;

        // This will adjust as needed per iteration
        // loop_rate.sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    sensor_msgs::JointState joint_state;
    ros::Rate loop_rate(30);

    // tf::TransformBroadcaster broadcaster;

    // const double degree = M_PI/180;
    // leg_control_data_lcmt tem_msg;
    // tem_msg.q[0] = 0;

    // robot state
    //double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
    double q[12] = {0};
    joint_state.header.stamp = ros::Time::now();
        // joint_state.name.resize(3);
        // joint_state.position.resize(3);
        // joint_state.name[0] ="swivel";
        // joint_state.position[0] = swivel;
        // joint_state.name[1] ="tilt";
        // joint_state.position[1] = tilt;
        // joint_state.name[2] ="periscope";
        // joint_state.position[2] = height;

    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.name[0] ="torso_to_abduct_fl_j";
    joint_state.position[0] = q[3];
    joint_state.name[1] ="abduct_fl_to_thigh_fl_j";
    joint_state.position[1] = q[4];
    joint_state.name[2] ="thigh_fl_to_knee_fl_j";
    joint_state.position[2] = q[5];
    joint_state.name[3] ="torso_to_abduct_fr_j";
    joint_state.position[3] = q[0];
    joint_state.name[4] ="abduct_fr_to_thigh_fr_j";
    joint_state.position[4] = q[1];
    joint_state.name[5] ="thigh_fr_to_knee_fr_j";
    joint_state.position[5] = q[2];
    joint_state.name[6] ="torso_to_abduct_hl_j";
    joint_state.position[6] = q[9];
    joint_state.name[7] ="abduct_hl_to_thigh_hl_j";
    joint_state.position[7] = q[10];
    joint_state.name[8] ="thigh_hl_to_knee_hl_j";
    joint_state.position[8] = q[11];
    joint_state.name[9] ="torso_to_abduct_hr_j";
    joint_state.position[9] = q[6];
    joint_state.name[10] ="abduct_hr_to_thigh_hr_j";
    joint_state.position[10] = q[7];
    joint_state.name[11] ="thigh_hr_to_knee_hr_j";
    joint_state.position[11] = q[8];

    // update transform
    // (moving in a circle with radius=2)

    // odom_trans.header.stamp = ros::Time::now();
    // odom_trans.transform.translation.x = cos(angle)*2;
    // odom_trans.transform.translation.y = sin(angle)*2;
    // odom_trans.transform.translation.z = .7;
    // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

    //send the joint state and transform
    joint_pub.publish(joint_state);

    // message declarations

    // geometry_msgs::TransformStamped odom_trans;
    ros::Subscriber sub = n.subscribe("/lcm_to_ros/leg_control_data", 1000, legCallback);

    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "axis";

    while (ros::ok()) {
        //update joint_state
        ros::spinOnce();
        loop_rate.sleep();

    }
    //ros::spin();


    return 0;
}

