#ifndef FUSE_VO_H
#define FUSE_VO_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

namespace fuse_vo {
	class FuseVO {
	public:
		FuseVO(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~FuseVO();

	private:
		LeftPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose);
		RightPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose);

	private:
		ros::Subscriber left_pose_sub;
		ros::Subscriber right_pose_sub;

		ros::Publisher robot_pose_pub;

		Eigen::Matrix3d left_pose_rotation;
		Eigen::Matrix1d left_pose_translation;
		Eigen::Matrix3d right_pose_rotation;
		Eigen::Matrix1d right_pose_translation;
		Eigen::Matrix4d left_to_robot;
		Eigen::Matrix4d right_to_robot;
	}
}
