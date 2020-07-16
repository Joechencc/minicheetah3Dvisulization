
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "leg_control_data_lcmt.hpp"
#include <lcm/lcm-cpp.hpp>


class Handler {
  public:
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const leg_control_data_lcmt *msg)
    {	
		ROS_INFO("q0:%f", msg->q[0]);
		ROS_INFO("q1:%f", msg->q[1]);
		ROS_INFO("q2:%f", msg->q[2]);
		ROS_INFO("q3:%f", msg->q[3]);

		ROS_INFO("qd0:%f", msg->q[0]);
		ROS_INFO("qd1:%f", msg->qd[1]);
		ROS_INFO("qd2:%f", msg->qd[2]);
		ROS_INFO("qd3:%f", msg->qd[3]);

        // spi_data_t my_data;
        // my_data.q_abad[0] = 0.1;
        // my_data.q_abad[1] = 0.2;
        // my_data.q_abad[2] = 0.3;
        // my_data.q_abad[3] = 0.4;

        // my_data.q_hip[0] = 0.1;
        // my_data.q_hip[1] = 0.2;
        // my_data.q_hip[2] = 0.3;
        // my_data.q_hip[3] = 0.4;

        // my_data.q_knee[0] = 1;
        // my_data.q_knee[1] = 0;
        // my_data.q_knee[2] = 0;
        // my_data.q_knee[3] = 0;

        // my_data.qd_abad[0] = 0.2;
        // my_data.qd_abad[1] = 0.2;
        // my_data.qd_abad[2] = 0.3;
        // my_data.qd_abad[3] = 0.1;

        // my_data.qd_hip[0] = 0.2;
        // my_data.qd_hip[1] = 0.2;
        // my_data.qd_hip[2] = 0.3;
        // my_data.qd_hip[3] = 0.1;

        // my_data.qd_knee[0] = 0.2;
        // my_data.qd_knee[1] = 0.2;
        // my_data.qd_knee[2] = 0.3;
        // my_data.qd_knee[3] = 0.1;
        
        // my_data.flags[0] = 1;
        // my_data.flags[1] = 1;
        // my_data.flags[2] = 1;
        // my_data.flags[3] = 1;
        
        // lcm::LCM lcm;

        // if (!lcm.good()){
        //     return;
        // }

        // my_data.spi_driver_status = 1;
        // sleep(1);
        // lcm.publish("spi_data", &my_data);

    }
};


int main(int argc, char **argv)
{
	//lcm in
    lcm::LCM lcm;

	if (!lcm.good())
		return 1;

	Handler handlerObject;
	lcm.subscribe("leg_control_data_lcmt", &Handler::handleMessage, &handlerObject);

	while (0 == lcm.handle()) {
	// printf("test!\n");
		// Do nothing
	}
  //ROS out
  
//   ros::init(argc, argv, "listener");

//   ros::NodeHandle n;

//   ros::Subscriber sub = n.subscribe("leg_control_data_lcmt", 1000, chatterCallback);

//   ros::spin();

//   return 0;
}
