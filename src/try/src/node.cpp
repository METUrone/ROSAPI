#include <ros/ros.h>
#include "drone_library/Drone.hpp"

int main(int argc, char **argv){
    ros::init(argc,argv,"node1");
    ros::NodeHandle nh;
    Drone dron(nh);
    dron.move_relative(2,0,0,0);
}
