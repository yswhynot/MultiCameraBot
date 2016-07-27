#include "multicam_calib_ros/multicam_calib_ros.h"

int main(int argc, char **argv)
{
  // Set up ROS.
	ros::init(argc, argv, "multicam_calib_ros");
	ros::NodeHandle nh;

  // Create a new node_example::Talker object.
	Multicambot::MultiCamCalib node(nh);

  // Let ROS handle all callbacks.
	ros::spin();

	return 0;
} 