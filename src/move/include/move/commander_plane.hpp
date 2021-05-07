#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointReached.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <move/PositionCommand.h>
#include <move/Battery.h>
#include <move/Position.h>

class UAV{
    private:
    // Variables for holding the current state of the vehicle
    static mavros_msgs::State current_state;
    static sensor_msgs::BatteryState battery;
    static geometry_msgs::PoseStamped pose;

    public:
    // ROS Things
    ros::ServiceClient client_takeoff,client_land,client_set_mode,client_arm,client_param_get,client_param_set;
    ros::Subscriber WP_sub,state_sub,battery_status_sub,position_sub;
    ros::Publisher rel_pos_cmd_pub;
    ros::ServiceServer server_battery,server_global_move_command,server_relative_move_command,server_position;

    UAV(ros::NodeHandle n);

    geometry_msgs::Point getPosition(void);

    sensor_msgs::BatteryState getBattery(void);

    mavros_msgs::State getState(void);

    bool parameterSet(const char* param_id,int integer,float real);

    bool parameterGet(const char* param_id,mavros_msgs::ParamValue& response);

    bool setTakeoff(float altitude);

    bool setLand(float altitude);

    bool set_mode(const char* mode);

    bool arm(bool state);

    bool get_failsafe(bool& DLL,bool& RCL);

    bool set_failsafe(bool DLL,bool RCL);

    bool takeoff(float altitude,bool blocking);

    static void WP_callback(const mavros_msgs::WaypointReached::ConstPtr& msg);

    static void state_tracker(const mavros_msgs::State::ConstPtr& _current_state);

    static void battery_tracker(const sensor_msgs::BatteryState::ConstPtr& _battery);

    static void pose_tracker(const geometry_msgs::PoseStamped::ConstPtr& _pose);

    static bool service_get_position(move::Position::Request &req,move::Position::Response &res);
    
    static bool service_command_global_position(move::PositionCommand::Request &req,move::PositionCommand::Response &res);

    static bool service_command_relative_position(move::PositionCommand::Request &req,move::PositionCommand::Response &res);

    static bool service_get_battery(move::Battery::Request &req,move::Battery::Response &res); 
};