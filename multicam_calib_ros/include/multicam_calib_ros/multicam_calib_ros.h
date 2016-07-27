#ifndef MULTICAM_CALIB_ROS_H
#define MULTICAM_CALIB_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include <vector>

namespace Multicambot {
	const int CAM_NUM = 5;

	struct ImgContainer {
		ImgContainer() {image_vec.reserve(CAM_NUM);}
		vector<Mat> image_vec;
		bool isFull() {
			bool is_full = true;
			for(int i = 0; i < CAM_NUM; i++) {
				if(image_vec[i].empty())
					is_full = false;
			}
			return is_full;
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