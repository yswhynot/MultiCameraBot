#include <fuse_vo/fuse_vo.h>

namespace fuse_vo {
	FuseVO::FuseVO(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
		
		// Subscribe
		left_pose_sub = nh.subscribe("/multicambot/camera_left0/pose", 1, &FuseVO::LeftPoseCallback, this);
		right_pose_sub = nh.subscribe("/multicambot/camera_right0/pose", 1, &FuseVO::RightPoseCallback, this);

		// Init camera-robot rotation & translation
		right_to_robot << -1, 0, 0,
						0, 0, -1,
						0, -1, 0;
		left_to_robot << -1, 0, 0,
						0, 0, -1,
						0, -1, 0;
	}
	
	FuseVO::LeftPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose) {

	}

	FuseVO::RightPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose) {

	}
}