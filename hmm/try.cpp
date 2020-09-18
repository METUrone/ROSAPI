#include <hmm/Drone.h>
#include <ros/ros.h>
int main(int argc ,char **argv){
	Drone dr;
	
	ros::init(argc,argv,"try");
	ros ::NodeHandle nh;
	ROS_INFO_STREAM(dr.i);
}
