#include "Drone.hpp"

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
    ros::spinOnce();
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
    ros::spinOnce();
}
void Drone::state_cb(const mavros_msgs::State::ConstPtr& msg){
    _current_state = *msg;
}

bool Drone::is_armed(){
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        if( _current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( _set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                        ROS_INFO("Offboard enabled");
            last_request = ros::Time::now();
            }
        else{
            if( !_current_state.armed && 
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( _arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                            return true;
                        }
                }
        }
    }
    return false;//why ?

}