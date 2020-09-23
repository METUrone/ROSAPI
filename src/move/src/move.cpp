#include<cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>

#include <move/Pos.h>
#include <move/Battery.h>
#include <move/Rot.h>

long double x = 0;
long double y = 0;
long double z = 2;
long double t = 0;
long double x_o = 0;
long double y_o = 0;
long double z_o = 0;
long double w_o = 0;

bool getService_global(
    move::Pos::Request &req,
    move::Pos::Response &res
) {

    ROS_WARN_STREAM("Move service is called");
    x = req.x;
    y = req.y;
    z = req.z;
    t = req.t;
    return true;
}
bool getService_rotate(
    move::Rot::Request &req,
    move::Rot::Response &res
) {
    //angular
	float x_local=0;
	float y_local=(3.141593/180)*req.y/2;
	float z_local=0;

    ROS_WARN_STREAM("Move service is called");
    x_o = sin(x_local)*cos(y_local)*cos(z_local)+cos(x_local)*sin(y_local)*sin(z_local);
    y_o = sin(x_local)*sin(y_local)*cos(z_local)+cos(x_local)*cos(y_local)*sin(z_local);
    z_o = cos(x_local)*sin(y_local)*cos(z_local)-sin(x_local)*cos(y_local)*sin(z_local);
    w_o = cos(x_local)*cos(y_local)*cos(z_local)-sin(x_local)*sin(y_local)*sin(z_local);
    return true;
}
bool getService_relative(
    move::Pos::Request &req,
    move::Pos::Response &res
) {
    ROS_WARN_STREAM("Move service is called");
    x += req.x;
    y += req.y;
    z += req.z;
    t += req.t;
    return true;
}

sensor_msgs::BatteryState battery;
bool getService_battery(
    move::Battery::Request &req,
    move::Battery::Response &res
) {
    ROS_WARN_STREAM("Battery service is called");
    res.voltage = battery.voltage;
    res.current = battery.current;
    res.remaining = battery.percentage;
    return true;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void battery_st(const sensor_msgs::BatteryState::ConstPtr& _battery){
    battery = *_battery;
}
int main(int argc, char **argv){
    
    ros::init(argc, argv, "move_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber battery_status = nh.subscribe<sensor_msgs::BatteryState>
            ("mavros/battery",10, battery_st);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceServer server_battery = nh.advertiseService(
        "Battery_status",
        &getService_battery
    );
    ros::ServiceServer server_global_move = nh.advertiseService(
        "Pos_global",
         &getService_global
    );
    ros::ServiceServer server_global_rotate = nh.advertiseService(
        "Pos_rotate",
         &getService_rotate
    );
    ros::ServiceServer server_relative_move = nh.advertiseService(
        "Pos_relative",
         &getService_relative
    );
    //there should be a spinOnce, hope the other spinOnce's will be enough.

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

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

    while(ros::ok()){
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
		pose.pose.orientation.x = x_o;
        pose.pose.orientation.y = y_o;
        pose.pose.orientation.z = z_o;
        pose.pose.orientation.w = w_o;
            
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        //Here we are constantly publishing the position.
        //If there is a service call or published position, add positions to pose.pose.position.xyz, it will do.
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
