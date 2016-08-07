#include <fuse_VO/fuse_VO.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fuse_VO");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    fuse_vo::FuseVO detector(nh, pnh);
    ros::spin();
}
