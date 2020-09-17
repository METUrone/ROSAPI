#include "Drone.hpp"

//Take coordinates from GPS, and put them into our coordinates, x,y,z
Drone::Drone(){
    ros::NodeHandle nh;
    ;
}

void Drone::move(long double x, long double y, long double z, long double t){
    //When to rotate? At the start of the move, while move, or at the end of the move?

    //Need to check if armed
    if(!is_armed()){
        arm();
    }

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _posx + x;
    pose.pose.position.y = _posy + y;
    pose.pose.position.z = _posz + z;

    local_pos_pub.publish(pose);
    ros::SpinOnce();
}