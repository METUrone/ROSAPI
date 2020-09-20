#include <ros/ros.h>
#include "drone_library/Drone.hpp"

int main(int argc, char **argv){
    ros::init(argc,argv,"node1");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("constructor start");
    Drone dron(nh);
    ROS_INFO_STREAM("constructor success");
    ROS_INFO_STREAM("move_relative success");
    dron.move_relative(2,0,0,0);
    ROS_INFO_STREAM("move_relative success");
}
