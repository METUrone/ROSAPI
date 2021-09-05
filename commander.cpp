#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>

#include <move/PositionCommand.h>
#include <move/Battery.h>
#include <move/Position.h>
#include <move/Camera.h>
#include<vector>


// Holds the given command by the services provided
// Then sends the commands to drone in the main function
geometry_msgs::TwistStamped pose_command;



// Holds the pose of the drone


std::vector<geometry_msgs::PoseStamped> poses;

void setPoses(){
    for(int i=0;i<10;i++){
        geometry_msgs::PoseStamped pose;
        poses.push_back(pose);
    }
}

bool service_get_position(
    move::Position::Request &req,
    move::Position::Response &res
) {
    //res.x = pose0.pose.position.x;
    //res.y = pose0.pose.position.y;
    //res.z = pose0.pose.position.z;
    
    return true;
}

// When the service called,
// the requested x,y,z and t is put to the pose_command.
bool service_command_global_position(
    move::PositionCommand::Request &req,
    move::PositionCommand::Response &res
) {
    ROS_INFO_STREAM("Move service is called");
/*
    pose_command.pose.position.x = req.x;
    pose_command.pose.position.y = req.y;
    pose_command.pose.position.z = req.z;
*/
    return true;
}

// When the service called,
// the requested x,y,z and t is added to the pose_command.
bool service_command_relative_position(
    move::PositionCommand::Request &req,
    move::PositionCommand::Response &res
) {
    ROS_INFO_STREAM("Move service is called");
/*
    pose_command.pose.position.x += req.x;
    pose_command.pose.position.y += req.y;
    pose_command.pose.position.z += req.z;
*/
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

// This holds the current state of the drone
// Connected, the flight mode etc.
mavros_msgs::State current_state;

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
void pose_tracker0(const geometry_msgs::PoseStamped::ConstPtr& _pose){
    poses[0] = *_pose;
}
void pose_tracker1(const geometry_msgs::PoseStamped::ConstPtr& _pose){
    poses[1] = *_pose;
}
void pose_tracker2(const geometry_msgs::PoseStamped::ConstPtr& _pose){
    poses[2] = *_pose;
}
void pose_tracker3(const geometry_msgs::PoseStamped::ConstPtr& _pose){
    poses[3] = *_pose;
}
void pose_tracker4(const geometry_msgs::PoseStamped::ConstPtr& _pose){
    poses[4] = *_pose;
}



int main(int argc, char **argv){
    
    // Initializing node
    // ros::init(argc, argv, "commander");
    std::string uav_number_str = argv[1];
    ROS_INFO_STREAM(uav_number_str);
    int uav_number = std::stoi(uav_number_str);

    int dumb = 0;
    ros::init(dumb,NULL,"UAV" + uav_number_str);
    ros::NodeHandle nh;

    pose_command.twist.linear.x = 0;
    pose_command.twist.linear.y = 0;
    pose_command.twist.linear.z = 5;
    pose_command.twist.angular.x = 0;
    pose_command.twist.angular.y = 0;
    pose_command.twist.angular.z = 0;

    // Subscriber topics
    
    // Current state of the drone
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav" + uav_number_str +"/mavros/state", 10, state_tracker);
    // FPV_cam connection
	// ros::Subscriber camera_sub = nh.subscribe
    //        ("/iris_fpv_cam/usb_cam/image_raw", 10, frame_save);
    // Current battery of the drone
    ros::Subscriber battery_status = nh.subscribe<sensor_msgs::BatteryState>
            ("/uav" + uav_number_str +"/mavros/battery",10, battery_tracker);
    // Current pose of the drone

    
    setPoses();

    ros::Subscriber position0 = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/mavros/local_position/pose",10, pose_tracker0);
    ros::Subscriber position1 = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/mavros/local_position/pose",10, pose_tracker1);
    ros::Subscriber position2 = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav2/mavros/local_position/pose",10, pose_tracker2);
    ros::Subscriber position3 = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav3/mavros/local_position/pose",10, pose_tracker3);
    ros::Subscriber position4 = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav4/mavros/local_position/pose",10, pose_tracker4);

    // Advertised topics

    // Used for giving commands to drone.
    ros::Publisher relative_position_command_publisher = nh.advertise<geometry_msgs::TwistStamped>
            ("/uav" + uav_number_str +"/mavros/setpoint_velocity/cmd_vel", 10);

    ROS_INFO_STREAM("/uav" + uav_number_str +"/mavros/setpoint_velocity/cmd_vel");
    // Service Clients

    // Changes arming status
    ros::ServiceClient arm_command = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav" + uav_number_str +"/mavros/cmd/arming");
    // Set flight mode
    ros::ServiceClient set_flight_mode = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav" + uav_number_str +"/mavros/set_mode");
  
    // Service Servers
    // These services are for external call.
    // If you call these services from this file, you probably are doing something wrong.

    ros::ServiceServer server_battery = nh.advertiseService(
        "/uav" + uav_number_str + "/battery_status",
        &service_get_battery
    );
    ros::ServiceServer server_camera_frame = nh.advertiseService(
        "/uav" + uav_number_str + "/camera",
        &service_get_camera_frame
    );
    ros::ServiceServer server_global_move_command = nh.advertiseService(
        "/uav" + uav_number_str + "/position/command_global",
         &service_command_global_position
    );
    ros::ServiceServer server_relative_move_command = nh.advertiseService(
        "/uav" + uav_number_str + "/position/command_relative",
         &service_command_relative_position
    );
    ros::ServiceServer server_position = nh.advertiseService(
        "/uav" + uav_number_str + "/position/position",
        &service_get_position
    );

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    
    ROS_INFO_STREAM("came here");


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){ // The number of loops may be decreased probably.
        relative_position_command_publisher.publish(pose_command);
        ros::spinOnce();
        rate.sleep();
    }

    // Used for setting the flight mode
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";
    
    // Used for sending arm command 
    mavros_msgs::CommandBool arm;
    arm.request.value = true;

    ros::Time last_request_time = ros::Time::now();
    while(ros::ok()){
        ROS_INFO_STREAM("in loop");
        ROS_INFO_STREAM(current_state.mode);
        ROS_INFO_STREAM(current_state.armed);
        if( (current_state.mode != "AUTO.LOITER") && (ros::Time::now()-last_request_time > ros::Duration(5.0)) ){
            if( set_flight_mode.call(set_mode) && set_mode.response.mode_sent){
                ROS_INFO_STREAM("Offboard enabled");
            }
            last_request_time = ros::Time::now();
        }
        else{
            if( !current_state.armed && (ros::Time::now() - last_request_time > ros::Duration(5.0)) ){
                if( arm_command.call(arm) && arm.response.success){
                    ROS_INFO_STREAM("Vehicle armed");
                }
                last_request_time = ros::Time::now();
            }
        }

        //Here we are constantly publishing the position.
		relative_position_command_publisher.publish(pose_command);
        ROS_INFO_STREAM("pose 0 ");
        ROS_INFO_STREAM(poses[0].pose.position.z);
        ROS_INFO_STREAM("pose 1 ");
        ROS_INFO_STREAM(poses[1].pose.position.z);
    
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
