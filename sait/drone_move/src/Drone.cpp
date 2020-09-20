#include <drone_library/Drone.hpp>
#include <ros/duration.h>
#include <mavros_msgs/CommandTOL.h>
#include<msgs/move_relative_server.h>

//Take coordinates from GPS, and put them into our coordinates, x,y,z
Drone::Drone(ros::NodeHandle nh){
    _nh = nh;
    _state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,&Drone::state_cb,this); // https://answers.ros.org/question/282259/ros-class-with-callback-methods/
    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);
    while(ros::ok() && !_current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
}

void Drone::move_relative(long double x, long double y, long double z, long double t){
    // srv->req x,y,z,t srv->resp none can changed from msgs srv move_relative_server
    ROS_INFO_STREAM("sa");
    ros :: ServiceClient spawnClient=_nh.serviceClient<msgs::move_relative_server>("it_is_about_sending_a_message");
    msgs::move_relative_server::Request req;
    msgs::move_relative_server::Response resp;
    req.x=x;
    req.y=y;
    req.z=z;
    req.t=t;
    bool success=spawnClient.call(req,resp);
    if(success) ROS_INFO_STREAM("succes_call");
    ros::Rate rate(20.0);
    //When to rotate? At the start of the move, while move, or at the end of the move?
    /*
    //Need to check if armed
    if(!is_armed()){
        arm();
    }
    */

    // //Publish these coordinates to a new topic, or may be a service?
    /*
    ROS_INFO("Starting to send.");
    ros::Publisher local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _posx + x;
    pose.pose.position.y = _posy + y;
    pose.pose.position.z = _posz + z;
    while(ros::ok()){
        ROS_INFO("Sent.");
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    */
}
void Drone::state_cb(const mavros_msgs::State::ConstPtr& msg){
    _current_state = *msg;
} 
