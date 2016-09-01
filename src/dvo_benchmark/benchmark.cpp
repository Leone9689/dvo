#include <ros/ros.h>
#include <ros/console.h>

#include <dvo_benchmark/camera_dense_tracking.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_tracker");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    CameraDenseTracker dense_tracker(nh, nh_private);

    ROS_INFO("started camera_tracker...");

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
