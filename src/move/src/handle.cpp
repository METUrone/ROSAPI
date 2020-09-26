#include <ros/ros.h>
#include <move/Pos.h>
#include <move/Battery.h>
#include <move/vel.h>
#include <move/circle.h>
#include <move/Position.h>

// Has x,y,z members
move::Position::Response position(ros::NodeHandle nh){
    move::Position::Response res;
    move::Position::Request req;
    ros::ServiceClient client = nh.serviceClient<move::Position>("Position");
    bool succ = client.call(req,res);
    if(succ){
        ROS_INFO_STREAM("Succesfull");
        return res;
    }
    else{
        ROS_ERROR_STREAM("Unsuccessfull");
        return res;
    }

}

void circle(long double speed, long double radius, long double t, ros::NodeHandle nh){
    move::circle::Response res;
    move::circle::Request req;
    ros::ServiceClient client = nh.serviceClient<move::circle>("circle");
    req.speed = speed;
    req.radius = radius;
    req.t = t;
    bool succ = client.call(req,res);
    if(succ){
        ROS_INFO_STREAM("Successfull");
    }
    else{
        ROS_INFO_STREAM("Unsuccessfull");
    }
}

// Unnecessary function for now...
void velocity(long double _x, long double _y, long double _z, long double __x, long double __y, long double __z, long double _t, ros::NodeHandle nh){
    move::vel::Response res;
    move::vel::Request req;
    ros::ServiceClient client = nh.serviceClient<move::vel>("Vel");
    req.x_lin = _x;
    req.y_lin = _y;
    req.z_lin = _z;
    req.x_ang = __x;
    req.y_ang = __y;
    req.z_ang = __z;
    req.t = _t;
    bool succ = client.call(req,res);
    if(succ){
        ROS_INFO_STREAM("Successfull");
    }
    else{
        ROS_INFO_STREAM("Unsuccessfull");
    }
}

void move_global(long double _x, long double _y, long double _z, long double _t, ros::NodeHandle nh){
    move::Pos::Response res;
    move::Pos::Request req;
    ros::ServiceClient client = nh.serviceClient<move::Pos>("Pos_global");
    req.x = _x;
    req.y = _y;
    req.z = _z;
    req.t = _t;
    bool succ = client.call(req, res);
    if(succ){
        ROS_INFO_STREAM("Succesfull");
    }
    else{
        ROS_INFO_STREAM("Unsuccesfull");
    }
}

void move_relative(long double _x, long double _y, long double _z, long double _t,ros::NodeHandle nh){
    move::Pos::Response res;
    move::Pos::Request req;
    ros::ServiceClient client = nh.serviceClient<move::Pos>("Pos_relative");
    req.x = _x;
    req.y = _y;
    req.z = _z;
    req.t = _t;
    bool succ = client.call(req, res);
    if(succ){
        ROS_INFO_STREAM("Succesfull");
    }
    else{
        ROS_INFO_STREAM("Unsuccesfull");
    }
}

// move::Battery::Response has 3 important property for now.
// voltage, current, remaining
move::Battery::Response battery_status(ros::NodeHandle nh){
    move::Battery::Response res;
    move::Battery::Request req;
    ros::ServiceClient client = nh.serviceClient<move::Battery>("Battery_status");
    bool succ = client.call(req,res);
    if(succ){
        ROS_INFO_STREAM("Succesfull");
        return res;
    }
    else{
        ROS_ERROR_STREAM("Unsuccessfull");
        return res;
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "handle_node");
    ros::NodeHandle nh;
    ros::Rate rate(0.2);
    ROS_INFO_STREAM("Donguye geldi.");
    while(ros::ok()){
        circle(0.0001,5,20,nh);
        ROS_INFO_STREAM("Request sent to function.");
        std::cout << battery_status(nh).remaining << std::endl; 
        std::cout << position(nh).z << std::endl;
        rate.sleep();
    }
}
