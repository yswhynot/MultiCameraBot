#ifndef MULTICAM_CALIB_ROS_H
#define MULTICAM_CALIB_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include <vector>

namespace Multicambot {
	struct ImgContainer {
		ImgContainer() {image_vec.reserve(5);}
		vector<Mat> image_vec;
		bool isFull() {
			return !(image_vec[0].empty() || image_vec[1].empty()
				|| image_vec[2].empty() || image_vec[3].empty()
				|| image_vec[4].empty()); 
		}

	};

	class MultiCamCalib {
	public:
		MultiCamCalib(ros::NodeHandle nh);
		ImageCallback(const sensor_msgs::Image& image_raw, int index);
		Calibration();

	private:
		image_transport::ImageTransport it_;
		
		ros::Subscriber cam_front_sub_;
		ros::Subscriber cam_left0_sub_;
		ros::Subscriber cam_left1_sub_;
		ros::Subscriber cam_right0_sub_;
		ros::Subscriber cam_right1_sub_;

		ImgContainer img_container;
	}
}