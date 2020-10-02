#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include<vector>

#include <move/Pos.h>
#include <move/Battery.h>
#include <move/Rot.h>
#include <move/vel.h>
#include <move/circle.h>
#include <move/Position.h>
#include <move/Empty.h>
#include <move/camera_info.h>

using namespace std;


long double x = 0;
long double y = 0;
long double z = 2;
long double t = 0;
long double x_o = 0;
long double y_o = 0;
long double z_o = 0;
long double w_o = 0;
unsigned int _height;       
unsigned int _width;
unsigned int _step;  
unsigned int _is_bigendian ;  
vector<uint8_t> _data;
vector<string> _info;
bool camera_connected=false;


bool main_loop=true;

bool circle(
    move::circle::Request &req,
    move::circle::Response &res
) {
	ros::NodeHandle nh;
	ros::Publisher circle_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
	geometry_msgs::PoseStamped pose;


	
	main_loop=false;
	float vel_time=req.t;
	float angle=0;
	float radius=req.radius;
	float speed=req.speed;

	ros::Time vel_request = ros::Time::now();
	ros::Rate rate(100.0);
	while(ros::Time::now()-vel_request<ros::Duration(vel_time)&&!main_loop){
		angle+=speed*0.01;
		pose.pose.position.x = x+sin(angle)*radius;
		pose.pose.position.y = y+cos(angle)*radius;
		pose.pose.position.z = z;
		circle_pub.publish(pose);
		ros::spinOnce();}

    
	main_loop=true;
    	return true;
}

geometry_msgs::PoseStamped real_position;
bool getService_pos(
    move::Position::Request &req,
    move::Position::Response &res
) {
    res.x = real_position.pose.position.x;
    res.y = real_position.pose.position.y;
    res.z = real_position.pose.position.z;
    
    return true;
}

bool getService_vel(
    move::vel::Request &req,
    move::vel::Response &res
) {
	main_loop=false;
	ros::NodeHandle nh;
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	geometry_msgs::Twist vel;
    ROS_WARN_STREAM("Vel service is called");
    vel.linear.x = req.x_lin;
	vel.linear.y = req.y_lin;
	vel.linear.z = req.z_lin;
	vel.angular.x = req.x_ang;
	vel.angular.y = req.y_ang;
	vel.angular.z = req.z_ang;
	float vel_time=req.t;
	ros::Time vel_request = ros::Time::now();
	while(ros::Time::now()-vel_request<ros::Duration(vel_time)&&!main_loop){
		vel_pub.publish(vel);}

	main_loop=true;
        return true;
}
bool set_main_loop(
    move::Empty::Request &req,
    move::Empty::Response &res
) {


    main_loop=true;

    return true;
}
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

bool getService_camera(
    move::camera_info::Request &req,
    move::camera_info::Response &res
) {
	if(!camera_connected){
		ROS_ERROR_STREAM("You are using drone without a camera .Please use correct drone(3dr iris with fpv camera");}
	else{
	res.height=_height;
	res.width=_width;
	res.step=_step;
	res.is_bigendian=_is_bigendian ;
    res.data=_data;
	ROS_INFO_STREAM("data size = step*rows");
	ROS_INFO_STREAM("encoding RGB8");}
	
	
	
	
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void battery_st(const sensor_msgs::BatteryState::ConstPtr& _battery){
    battery = *_battery;
}

void camera_msg(const sensor_msgs::Image &_image){
	 camera_connected=true;
    _height=_image.height;
	_width=_image.width;
	_step=_image.step;   
	_is_bigendian=_image.is_bigendian ;
	_data=_image.data;



}

void position_func(const geometry_msgs::PoseStamped::ConstPtr& _pose){
    real_position = *_pose;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "move_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::Subscriber camera_sub = nh.subscribe
            ("/iris_fpv_cam/usb_cam/image_raw", 10, camera_msg);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber battery_status = nh.subscribe<sensor_msgs::BatteryState>
            ("mavros/battery",10, battery_st);
    ros::Subscriber position = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose",10, position_func);
  
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
  
    ros::ServiceServer server_battery = nh.advertiseService(
        "Battery_status",
        &getService_battery
    );
    ros::ServiceServer server_camera = nh.advertiseService(
        "Camera_info",
        &getService_camera
    );
    ros::ServiceServer server_global_move = nh.advertiseService(
        "Pos_global",
         &getService_global
    );
    ros::ServiceServer server_circle = nh.advertiseService(
        "circle",
        &circle
        );
    ros::ServiceServer server_global_rotate = nh.advertiseService(
        "Pos_rotate",
         &getService_rotate
    );
    ros::ServiceServer server_relative_move = nh.advertiseService(
        "Pos_relative",
         &getService_relative
    );
	ros::ServiceServer server_vel = nh.advertiseService(
        "Vel",
         &getService_vel
    );
    ros::ServiceServer server_position = nh.advertiseService(
        "Position",
        &getService_pos
    );
    ros::ServiceServer server_set_main_loop = nh.advertiseService(
        "exit_call",
        &set_main_loop
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
        if(main_loop){
			local_pos_pub.publish(pose);}
    
        //std::cout << real_position.pose.position.x << " " << real_position.pose.position.y << " " << real_position.pose.position.z << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
