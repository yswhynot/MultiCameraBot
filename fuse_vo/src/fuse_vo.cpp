#include <fuse_vo/fuse_vo.h>

#include <Eigen/Geometry>

namespace fuse_vo {
	FuseVO::FuseVO(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
		
		// Subscribe
		left_pose_sub = nh.subscribe("/multicambot/camera_left0/pose", 1, &FuseVO::LeftPoseCallback, this);
		right_pose_sub = nh.subscribe("/multicambot/camera_right0/pose", 1, &FuseVO::RightPoseCallback, this);

		// Init camera-robot rotation & translation
		Eigen::MatrixXd left_to_robot_tmp(3, 4);
		Eigen::MatrixXd right_to_robot_tmp(3, 4);
		right_to_robot_tmp << -1, 0, 0, 0,
		0, 0, 1, 0,
		0, -1, 0, 0.08;
		left_to_robot_tmp << -1, 0, 0, 0,
		0, 0, -1, 0,
		0, -1, 0, -0.08;
		left_to_robot = left_to_robot_tmp;
		right_to_robot = right_to_robot_tmp;
	}
	
	void FuseVO::LeftPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose) {
		geometry_msgs::PoseStamped pose = *input_pose;
		left_pose_translation << pose.position.x, pose.position.y, pose.position.z;
		GeoPoseToMat(pose.orientation, left_pose_rotation);
	}

	void FuseVO::RightPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose) {
		geometry_msgs::PoseStamped pose = *input_pose;
		right_pose_translation << pose.position.x, pose.position.y, pose.position.z;
		GeoPoseToMat(pose.orientation, right_pose_rotation);
	}

	void FuseVO::GeoPoseToMat(geometry_msgs::Quaternion& input_q, Eigen::Matrix3d& output) {
		Eigen::Quaternion<double> q(input_q.w, input_q.x, input_q.y, input_q.z);
		Eigen::Matrix3d rotation = q.matrix();

		output = rotation;
	}

	void FuseVO::TransformLeftPose() {
		Eigen::Vector6d twist;
		Eigen::Matrix6d adg;
		MatToVec6d(left_pose_rotation, left_pose_translation, twist);
		ComputeAdjointTransform(left_pose_rotation, left_pose_translation, adg);


	}

	void FuseVO::TransformRightPose() {
		Eigen::Vector6d twist;
		Eigen::Matrix6d adg;
		MatToVec6d(right_pose_rotation, right_pose_translation, twist);
		ComputeAdjointTransform(right_pose_rotation, right_pose_translation, adg);
	}


	void FuseVO::ComputeAdjointTransform(Eigen::Matrix3d& R, Eigen::Vector3d& p, Eigen::Matrix6d& output) {
		Eigen::Matrix6d adg;
		Eigen::Matrix3d p_mat;

		float w1 = p(0);
		float w2 = p(1);
		float w3 = p(2);

		p_mat << 0, -w3, w2,
		w3, 0, -w1,
		-w2, w1, 0;


		Eigen::Matrix3d pR = p_mat * R;

		adg.block<3, 3>(0, 0) = R;
		adg.block<3, 3>(0, 3) = pR;
		adg.block<3, 3>(3, 0) = 0;
		adg.block<3, 3>(3, 3) = R;

		output = adg;

	}

	void FuseVO::MatToVec6d(Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::Vector6d& output) {
		Eigen::Vector6d twist;
		Eigen::Vector3d R_vec = R.eulerAngles(0, 1, 2);
		twist.head(3) = t;
		twist.tail(3) = R_vec;

		output = twist;
	}

	void FuseVO::Vec6dToMat(Eigen::Vector6d& twist, Eigen::Matrix3d& output_R, Eigen::Vector3d& output_t) {
		Eigen::AngleAxisf roll(twist(3), Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf yaw(twist(4), Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf pitch(twist(5), Eigen::Vector3f::UnitZ());

		Eigen::Quaternion<double> q = roll * yaw * pitch;
		Eigen::Matrix3d rotation = q.matrix();
 
		output_R = rotation;
		output_t = twist.tail(3);
	}
}