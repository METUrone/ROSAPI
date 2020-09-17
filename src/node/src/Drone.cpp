#include "Drone.hpp"

//Take coordinates from GPS, and put them into our coordinates, x,y,z
Drone::Drone(ros::NodeHandle nh){
    _nh = nh;
    _state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);
    while(ros::ok() && !_current_state.connected){
        ros::SpinOnce();
        rate.sleep();
    }
}

void Drone::move_relative(long double x, long double y, long double z, long double t){
    //When to rotate? At the start of the move, while move, or at the end of the move?

    //Need to check if armed
    if(!is_armed()){
        arm();
    }

    ros::Publisher local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _posx + x;
    pose.pose.position.y = _posy + y;
    pose.pose.position.z = _posz + z;

    local_pos_pub.publish(pose);
    ros::SpinOnce();
}

void Drone::move_global(long double x, long double y, long double z, long double t){
    //When to rotate? At the start of the move, while move, or at the end of the move?

    //Need to check if armed
    if(!is_armed()){
        arm();
    }

    ros::Publisher local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    local_pos_pub.publish(pose);
    ros::SpinOnce();
}
void Drone::state_cb(const mavros_msgs::State::ConstPtr& msg){
    _current_state = *msg;
}