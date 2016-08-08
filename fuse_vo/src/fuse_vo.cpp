#include <fuse_vo/fuse_vo.h>

#include <Eigen/Geometry>
#include <iostream>

using namespace std;

namespace fuse_vo {
	FuseVO::FuseVO(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
		
		// Subscribe
		left_pose_sub = nh.subscribe("/multicambot/camera_left0/pose", 1, &FuseVO::LeftPoseCallback, this);
		right_pose_sub = nh.subscribe("/multicambot/camera_right0/pose", 1, &FuseVO::RightPoseCallback, this);

		// Init camera-robot rotation & translation
		right_to_robot << 1, 0, 0, 0,
		0, 0, 1, 0,
		0, 1, 0, -0.08, 
		0, 0, 0, 1;
		left_to_robot << -1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 0.08,
		0, 0, 0, 1;
	}

	FuseVO::~FuseVO() {
		left_pose_sub.shutdown();
		right_pose_sub.shutdown();
	}
	
	void FuseVO::LeftPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose) {
		geometry_msgs::PoseStamped pose = *input_pose;

		Eigen::Matrix3d rotation;
		GeoPoseToMat(pose.pose.orientation, rotation);
		if(rotation.row(0).norm() > 1.1 || rotation.row(1).norm() > 1.1 || rotation.row(2).norm() > 1.1)
			return;

		left_pose_rotation = rotation;
		left_pose_translation << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;

		cout << "Left pose:\n" << left_pose_rotation << endl;
		cout << "Left trans:\n" << left_pose_translation <<endl;

		TransformLeftPose();
	}

	void FuseVO::RightPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose) {
		geometry_msgs::PoseStamped pose = *input_pose;

		Eigen::Matrix3d rotation;
		GeoPoseToMat(pose.pose.orientation, rotation);
		if(rotation.row(0).norm() > 1.1 || rotation.row(1).norm() > 1.1 || rotation.row(2).norm() > 1.1)
			return;

		right_pose_rotation = rotation;
		right_pose_translation << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;

		cout << "Right pose:\n" << right_pose_rotation << endl;
		cout << "Right trans:\n" << right_pose_translation << endl;

		TransformRightPose();
	}

	void FuseVO::GeoPoseToMat(geometry_msgs::Quaternion& input_q, Eigen::Matrix3d& output) {
		Eigen::Quaternion<double> q(input_q.w, input_q.x, input_q.y, input_q.z);
		Eigen::Matrix3d rotation = q.matrix();

		output = rotation;
	}

	void FuseVO::TransformLeftPose() {
		Eigen::VectorXd twist;
		Eigen::MatrixXd adg;
		MatToVec6d(left_pose_rotation, left_pose_translation, twist);
		ComputeAdjointTransform(left_pose_rotation, left_pose_translation, adg);

		Eigen::VectorXd body_twist = adg * twist;
		Eigen::Matrix4d new_pose;
		Vec6dToMat(body_twist, new_pose);

		if(new_pose(0,3) > 2 || new_pose(1,3) > 2 || new_pose(2,3) > 2)
			return;
		trans_pose_left = new_pose;

		cout << "trans_pose_left: \n" << trans_pose_left << endl;
	}

	void FuseVO::TransformRightPose() {
		Eigen::VectorXd twist;
		Eigen::MatrixXd adg;
		MatToVec6d(right_pose_rotation, right_pose_translation, twist);
		ComputeAdjointTransform(right_pose_rotation, right_pose_translation, adg);

		Eigen::VectorXd body_twist = adg * twist;
		Eigen::Matrix4d new_pose;
		Vec6dToMat(body_twist, new_pose);

		if(new_pose(0,3) > 2 || new_pose(1,3) > 2 || new_pose(2,3) > 2)
			return;
		trans_pose_right = new_pose;

		cout << "trans_pose_right: \n" << trans_pose_right << endl;
	}


	void FuseVO::ComputeAdjointTransform(Eigen::Matrix3d& R, Eigen::Vector3d& p, Eigen::MatrixXd& output) {
		Eigen::MatrixXd adg(6, 6);
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
		adg.block<3, 3>(3, 0).setZero();
		adg.block<3, 3>(3, 3) = R;

		output = adg;
	}

	void FuseVO::MatToVec6d(Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::VectorXd& output) {
		Eigen::VectorXd twist(6);
		Eigen::Vector3d R_vec = R.eulerAngles(0, 1, 2);
		twist.head(3) = t;
		twist.tail(3) = R_vec;

		output = twist;
	}

	void FuseVO::Vec6dToMat(Eigen::VectorXd& twist, Eigen::Matrix4d& output) {
		Eigen::AngleAxisd roll(twist(3), Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd yaw(twist(4), Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd pitch(twist(5), Eigen::Vector3d::UnitZ());

		Eigen::Quaternion<double> q = roll * yaw * pitch;
		Eigen::Matrix3d rotation = q.matrix();

		Eigen::Matrix4d affine;
		affine.block<3, 3>(0, 0) = rotation;
		affine.block<3, 1>(0, 3) = twist.tail(3);
		affine.block<1, 3>(3, 0).setZero();
		affine(3, 3) = 1;

		output = affine;
	}

}