#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <move/PositionCommand.h>
#include <move/Battery.h>
#include <move/Position.h>
#include <move/Euler.h>
#include <move/Camera.h>
#include <move/ArmDisarmCommand.h>
#include <move/TkoffLandCommand.h>
#include <move/State.h>

#include "move/Mesafe.h"
#include "move/MesafeResponse.h"

#include <math.h> 

#include <cstdlib>
#define _USE_MATH_DEFINES
#include <cmath>

#define TRY(__function__) for(int i = 0; (!__function__) && (i < 5) ;i++) {ros::Duration(1.0).sleep(); ROS_INFO("Trying Again");}

#define GCS_MODE "GUIDED" // "OFFBOARD" for PX4 | "GUIDED" for ARDUPILOT
#define OFFSET_ANGLE eang.yaw // yaw_angle for auto heading angle offset set at arming // in radians

struct EulerAngles {
    double pitch, roll, yaw;
};

EulerAngles eang;

EulerAngles QuaternionToEuler(geometry_msgs::Quaternion& q) {
    EulerAngles res;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    res.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        res.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        res.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    res.yaw = std::atan2(siny_cosp, cosy_cosp);
    return res;
}

// Function for converting radian to degree
double Convert(double radian)
{
    return(radian * (180 / M_PI));
}

geometry_msgs::PoseStamped rotate(geometry_msgs::PoseStamped& _pose, double _yaw){
    geometry_msgs::PoseStamped res;
    res.pose.position.x = _pose.pose.position.x*std::cos(_yaw) - _pose.pose.position.y*std::sin(_yaw);
    res.pose.position.y = _pose.pose.position.x*std::sin(_yaw) + _pose.pose.position.y*std::cos(_yaw);
    res.pose.position.z = _pose.pose.position.z;
    return res;
}

// Holds the given command by the services provided
// Then sends the commands to drone in the main function
geometry_msgs::PoseStamped pose_command;


// Holds the pose of the drone
geometry_msgs::PoseStamped pose;
bool poseRecieved = false;

// Responses the position of the drone, x,y,z when the service is called.
bool service_get_position(
    move::Position::Request &req,
    move::Position::Response &res
) {
    geometry_msgs::PoseStamped rotated_pose = rotate(pose,-OFFSET_ANGLE);
    res.x = rotated_pose.pose.position.x;
    res.y = rotated_pose.pose.position.y;
    res.z = rotated_pose.pose.position.z;
    
    return true;
}

ros::Publisher relative_position_command_publisher;
// When the service called,
// the requested x,y,z and t is put to the pose_command.
bool service_command_global_position(
    move::PositionCommand::Request &req,
    move::PositionCommand::Response &res
) {
    ROS_INFO_STREAM("Move service is called");

    pose_command.pose.position.x = req.x;
    pose_command.pose.position.y = req.y;
    pose_command.pose.position.z = req.z;

    relative_position_command_publisher.publish(rotate(pose_command,OFFSET_ANGLE)); // calculates position according to the drone's body frame instead of NED
    ROS_INFO("%.2f %.2f %.2f",pose_command.pose.position.x,pose_command.pose.position.y,pose_command.pose.position.z);
    return true;
}

// When the service called,
// the requested x,y,z and t is added to the pose_command.
bool service_command_relative_position(
    move::PositionCommand::Request &req,
    move::PositionCommand::Response &res
) {
    ROS_INFO_STREAM("Move service is called");

    pose_command.pose.position.x += req.x;
    pose_command.pose.position.y += req.y;
    pose_command.pose.position.z += req.z;

    relative_position_command_publisher.publish(rotate(pose_command,OFFSET_ANGLE)); // calculates position according to the drone's body frame instead of NED
    ROS_INFO("%.2f %.2f %.2f",pose_command.pose.position.x,pose_command.pose.position.y,pose_command.pose.position.z);
    return true;
}

// Holds the battery state.
sensor_msgs::BatteryState battery;

// Responses the voltage, current and remaining percentage of battery when the service called.
bool service_get_battery(
    move::Battery::Request &req,
    move::Battery::Response &res
) {
    ROS_INFO_STREAM("Battery service is called");

    res.voltage = battery.voltage;
    res.current = battery.current;
    res.remaining = battery.percentage;

    return true;
}

// Holds the camera frame if there is a camera connected and running.
sensor_msgs::Image frame;

// True if camera is connected, false otherwise.
bool camera_connected=false;

// Responses the frame of the camera
// In response, gives height and width of the frame,
// frame step, the representation is big endian or not, and the actual data.
bool service_get_camera_frame(
    move::Camera::Request &req,
    move::Camera::Response &res
) {
	if(!camera_connected){
		ROS_ERROR_STREAM("You are using drone without a camera. Please use correct drone(3dr iris with fpv camera");
    }
	else{
	res.height= frame.height;
	res.width= frame.width;
	res.step= frame.step;
	res.is_bigendian= frame.is_bigendian ;
    res.data= frame.data;
	ROS_INFO_STREAM("Data size = step*rows"); //WTF is that really working?
	ROS_INFO_STREAM("Encoding RGB8");
    }	
}

ros::ServiceClient arm_command;

ros::ServiceClient tkoff_command;

ros::ServiceClient land_command;

bool arm_request_state;

bool service_command_arm_disarm(move::ArmDisarmCommand::Request &req, move::ArmDisarmCommand::Response &res)
{
    /* CommandBool
    bool value
    ---
    bool success
    uint8 result
    */
    
    mavros_msgs::CommandBool srv;
    srv.request.value = req.cmd;
    if (arm_command.call(srv))
    {
        if(srv.response.success){
            if(req.cmd){
                eang = QuaternionToEuler(pose.pose.orientation);
                ROS_INFO("YAW ANGLE: %.2f", Convert(eang.yaw));
            }
            ROS_INFO("Vehicle %sarmed!", req.cmd ? "" : "dis");
            return true;
        }else{
            ROS_WARN("Failed to %sarm!", req.cmd ? "" : "dis");
        }
    }else{
        ROS_ERROR("Failed to call service arm!");
    }
    return false;
    
    arm_request_state = req.cmd;
}

bool service_command_tkoff_land(move::TkoffLandCommand::Request &req, move::TkoffLandCommand::Response &res)
{
    /* CommandTOL
    float32 min_pitch # used by takeoff
    float32 yaw
    float32 latitude
    float32 longitude
    float32 altitude
    ---
    bool success
    uint8 result
    */
    
    mavros_msgs::CommandTOL srv;
    if(req.cmd){
        srv.request.altitude = req.altitude;
        srv.request.yaw = eang.yaw;
        if (tkoff_command.call(srv))
        {
            if(srv.response.success){
                ROS_INFO("Vehicle taking off!");
                return true;
            }else{
                ROS_WARN("Failed to takeoff!");
            }
        }else{
            ROS_ERROR("Failed to call service takeoff!");
        }
    }else{
        if (land_command.call(srv))
        {
            if(srv.response.success){
                ROS_INFO("Vehicle landing!");
                return true;
            }else{
                ROS_WARN("Failed to land!");
            }
        }else{
            ROS_ERROR("Failed to call service land!");
        }
    }
    return false;
    
}

// This holds the current state of the drone
// Connected, the flight mode etc.
mavros_msgs::State current_state;

bool service_get_state(move::State::Request &req,move::State::Response &res)
{
    res.onAir = current_state.armed && pose.pose.position.z > 0.1;
    res.armed = current_state.armed;
    res.flightMode = current_state.mode;
}


// Saves the state of the drone
void state_tracker(const mavros_msgs::State::ConstPtr& _current_state){
    current_state = *_current_state;
}

// Saves the battery state of the drone
void battery_tracker(const sensor_msgs::BatteryState::ConstPtr& _battery){
    battery = *_battery;
}

// Saves the frames of the FPV camera
void frame_save(const sensor_msgs::Image &_frame){
	frame = _frame;
    camera_connected=true; // Are there any other way to detect if camera is running?
}

// Saves the pose of the drone
void pose_tracker(const nav_msgs::Odometry::ConstPtr& _odometry){
    pose.pose = _odometry->pose.pose;
    poseRecieved = true;
}




int main(int argc, char **argv){
    
    // Initializing node
    ros::init(argc, argv, "commander");
    ros::NodeHandle nh;

    // Subscriber topics
    
    // Current state of the drone
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_tracker);
    // FPV_cam connection
	ros::Subscriber camera_sub = nh.subscribe
            ("/iris_fpv_cam/usb_cam/image_raw", 10, frame_save);
    // Current battery of the drone
    ros::Subscriber battery_status = nh.subscribe<sensor_msgs::BatteryState>
            ("mavros/battery",10, battery_tracker);
    // Current pose of the drone
    ros::Subscriber position = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local",10, pose_tracker);

    // Advertised topics

    // Used for giving commands to drone.
    relative_position_command_publisher = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // Service Clients

    // Changes arming statu
    arm_command = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    
    tkoff_command = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");

    land_command = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    // Set flight mode
    ros::ServiceClient set_flight_mode = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
  
    // Service Servers
    // These services are for external call.
    // If you call these services from this file, you probably are doing something wrong.

    ros::ServiceServer server_battery = nh.advertiseService(
        "battery_status",
        &service_get_battery
    );
    ros::ServiceServer server_camera_frame = nh.advertiseService(
        "camera",
        &service_get_camera_frame
    );
    ros::ServiceServer server_global_move_command = nh.advertiseService(
        "position/command_global",
         &service_command_global_position
    );
    ros::ServiceServer server_relative_move_command = nh.advertiseService(
        "position/command_relative",
         &service_command_relative_position
    );
    ros::ServiceServer server_position = nh.advertiseService(
        "position/position",
        &service_get_position
    );

    ros::ServiceServer server_arm_disarm = nh.advertiseService(
        "arm_disarm",
        &service_command_arm_disarm
    );

    ros::ServiceServer server_tkoff_land = nh.advertiseService(
        "move/cmd/tkoff_land",
        &service_command_tkoff_land
    );

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // wait for the first pose data from mavros topic
    while(ros::ok() && !poseRecieved){
        ros::spinOnce();
        rate.sleep();
    }
    pose_command = pose;// Reset to the current pose

    eang = QuaternionToEuler(pose.pose.orientation);
    ROS_INFO("YAW ANGLE: %.2f", Convert(eang.yaw));

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){ // The number of loops may be decreased probably.
        relative_position_command_publisher.publish(pose_command);
        ros::spinOnce();
        rate.sleep();
    }

    // Used for setting the flight mode
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = GCS_MODE;
    
    //TRY(set_flight_mode.call(set_mode) && set_mode.response.mode_sent);
    while(!(set_flight_mode.call(set_mode) && set_mode.response.mode_sent)){
        ROS_WARN("%s could not be enabled! Trying again..",GCS_MODE);
    }
    ROS_INFO("%s mode enabled",GCS_MODE);

    // Used for sending arm command 
    mavros_msgs::CommandBool arm;
    arm.request.value = true;
    rate = ros::Rate(20);
    ros::Time last_request_time = ros::Time::now();
    while(ros::ok()){
        if(current_state.mode != GCS_MODE){
            if (ros::Time::now()-last_request_time > ros::Duration(5.0)){
                ROS_WARN("%s mode disabled!",GCS_MODE);
                last_request_time = ros::Time::now();
            }
        }else{
            //Here we are constantly publishing the position.
		    //relative_position_command_publisher.publish(calculate_pose(pose_command, rotation_matrix)); // calculates position according to the drone's body frame instead of NED
            //ROS_INFO("%.2f %.2f %.2f",pose_command.pose.position.x,pose_command.pose.position.y,pose_command.pose.position.z);

            if( !current_state.armed && arm_request_state && (ros::Time::now() - last_request_time > ros::Duration(5.0)) ){
                if( arm_command.call(arm) && arm.response.success){
                    ROS_INFO_STREAM("Vehicle armed automaticly");
                }
                last_request_time = ros::Time::now();
            }
        }
        
    
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
