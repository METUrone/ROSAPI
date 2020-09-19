#include <drone_library/Drone.hpp>
#include <ros/duration.h>
#include <mavros_msgs/CommandTOL.h>

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
    ros::Rate rate(20.0);
    //When to rotate? At the start of the move, while move, or at the end of the move?
    /*
    //Need to check if armed
    if(!is_armed()){
        arm();
    }
    */
    arm();

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
}

void Drone::move_global(long double x, long double y, long double z, long double t){
    //When to rotate? At the start of the move, while move, or at the end of the move?
    /*
    //Need to check if armed
    if(!is_armed()){
        arm();
    }
    */
    arm();

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

bool Drone::is_armed(){// Needs to be completed
    return true;
}

void Drone::arm(){
    ros::Publisher local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    // If we do not send some positions before, somehow the requests are refused.
    // For more information, https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time takeoff_time = ros::Time::now();

    while(ros::ok()){
        if( _current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( _set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( _arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(ros::Time::now() - takeoff_time > ros::Duration(15.0)){
            ROS_INFO("Takeoff done");
            return;
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
}