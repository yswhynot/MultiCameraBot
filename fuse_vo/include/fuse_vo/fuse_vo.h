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
		void LeftPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose);
		void RightPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose);
		void GeoPoseToMat(geometry_msgs::Quaternion& input_q, Eigen::Matrix3d& output);
		void ComputeAdjointTransform(Eigen::Matrix3d& R, Eigen::Matrix1d& p, Eigen::MatrixXd& output);

	private:
		ros::Subscriber left_pose_sub;
		ros::Subscriber right_pose_sub;

		ros::Publisher robot_pose_pub;

		Eigen::Matrix3d left_pose_rotation;
		Eigen::Vector3d left_pose_translation;
		Eigen::Matrix3d right_pose_rotation;
		Eigen::Vector3d right_pose_translation;
		Eigen::MatrixXd left_to_robot;
		Eigen::MatrixXd right_to_robot;

		Eigen::Matrix4d trans_pose_left;
		Eigen::Matrix4d trans_pose_right;
	}
}
