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
		void ComputeAdjointTransform(Eigen::Matrix3d& R, Eigen::Vector3d& p, Eigen::MatrixXd& output);
		void TransformLeftPose();
		void TransformRightPose();
		void MatToVec6d(Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::VectorXd& output);
		void Vec6dToMat(Eigen::VectorXd& twist, Eigen::Matrix4d& output);

	private:
		ros::Subscriber left_pose_sub;
		ros::Subscriber right_pose_sub;

		ros::Publisher robot_pose_pub;

		Eigen::Matrix3d left_pose_rotation;
		Eigen::Vector3d left_pose_translation;
		Eigen::Matrix3d right_pose_rotation;
		Eigen::Vector3d right_pose_translation;
		Eigen::Matrix4d left_to_robot;
		Eigen::Matrix4d right_to_robot;

		Eigen::Matrix4d trans_pose_left;
		Eigen::Matrix4d trans_pose_right;
	};
}

#endif
