#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <multicam_calib_ros/multicam_calib_ros.h>

using namespace cv;
using namespace std;

namespace Multicambot {
	enum {LEFT1, LEFT0, FRONT, RIGHT0, RIGHT1};

	MultiCamCalib::MultiCamCalib(ros::NodeHandle nh) : it_(nh) {

		// subscribers
		cam_front_sub_ = it_.subscribe<sensor_msgs::Image> ("/multicambot/camera_front/image_raw", 1, boost::bind(ImageCallback, _1, FRONT));
		cam_front_sub_ = it_.subscribe<sensor_msgs::Image> ("/multicambot/camera_left1/image_raw", 1, boost::bind(ImageCallback, _1, LEFT1));
		cam_front_sub_ = it_.subscribe<sensor_msgs::Image> ("/multicambot/camera_left0/image_raw", 1, boost::bind(ImageCallback, _1, LEFT0));
		cam_front_sub_ = it_.subscribe<sensor_msgs::Image> ("/multicambot/camera_right0/image_raw", 1, boost::bind(ImageCallback, _1, RIGHT0));
		cam_front_sub_ = it_.subscribe<sensor_msgs::Image> ("/multicambot/camera_right1/image_raw", 1, boost::bind(ImageCallback, _1, RIGHT1));
	}

	void MultiCamCalib::ImageCallback(const sensor_msgs::Image& image_raw, int index) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::MONO8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		img_container.img_vec[index] = cv_ptr->image;

		if(img_container.isFull())
			Calibration();

	}

	void MultiCamCalib::Calibration() {
		
	}
}