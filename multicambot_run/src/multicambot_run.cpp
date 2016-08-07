#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <gazebo/gazebo.hh>
// #include <gazebo/ModelState.hh>

#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "multicambot_run");

	ros::NodeHandle* nh = new ros::NodeHandle();
	ros::Publisher pose_pub = nh->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

	gazebo_msgs::ModelState ms;
	ms.model_name = "multicambot";

	ros::Rate loop_rate(10);
	
	double theta = 0;
	const float RADIUS = 1;
	while (ros::ok()) {
		if(ms.pose.position.x < -3) {
			theta += 0.0785;
			
			ms.pose.position.x = -3 - sin(theta);
			ms.pose.position.y = 1 - cos(theta);

			ms.pose.orientation.z = -sin(theta / 2);	
			ms.pose.orientation.w = cos(theta / 2);
		} else {
			ms.pose.position.x += -0.03;
		}

		if(ms.pose.position.y > 1)
			break;

		pose_pub.publish(ms);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
