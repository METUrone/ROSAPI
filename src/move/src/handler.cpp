#include <ros/ros.h>
#include <move/Position.h>
#include <move/PositionCommand.h>
#include <move/Battery.h>

typedef struct{
    double x;
    double y;
    double z;
    bool success;
} position;

typedef struct{
    double voltage;
    double current;
    double remaining;
    bool success;
} battery;

position takePositionInfo(ros::NodeHandle nh){
    move::Position::Response res;
    move::Position::Response req;
    ros::ServiceClient client = nh.serviceClient<move::Position>("position/position");

    bool success = client.call(req,res);
    if(success){
        ROS_INFO_STREAM("TakePositionInfo call with success");
        position response = {res.x, res.y, res.z, true};
        return response;
    }
    else{
        ROS_INFO_STREAM("TakePositionInfo call with error");
        position response = {0, 0, 0, false};
        return response;
    }
}

bool moveGlobal(position pos, ros::NodeHandle nh){
    move::PositionCommand::Response res;
    move::PositionCommand::Request req;
    ros::ServiceClient client = nh.serviceClient<move::PositionCommand>("position/command_global");

    req.x = pos.x;
    req.y = pos.y;
    req.z = pos.z;

    bool success = client.call(req,res);
    if(success){
        ROS_INFO_STREAM("moveGlobal call with success");
        return true;
    }
    else{
        ROS_INFO_STREAM("moveGlobal call with error");
        return false;
    }
}

bool moveRelative(position pos, ros::NodeHandle nh){
    move::PositionCommand::Response res;
    move::PositionCommand::Request req;
    ros::ServiceClient client = nh.serviceClient<move::PositionCommand>("position/command_relative");

    req.x = pos.x;
    req.y = pos.y;
    req.z = pos.z;

    bool success = client.call(req,res);
    if(success){
        ROS_INFO_STREAM("moveRelative call with success");
        return true;
    }
    else{
        ROS_INFO_STREAM("moveRelative call with error");
        return false;
    }
}

battery batteryStatus(ros::NodeHandle nh){
    move::Battery::Response res;
    move::Battery::Request req;
    ros::ServiceClient client = nh.serviceClient<move::Battery>("battery_status");

    bool success = client.call(req,res);
    if(success){
        ROS_INFO_STREAM("batteryStatus call with success");
        battery response = {res.voltage, res.current, res.remaining, true};
        return response;
    }
    else{
        ROS_INFO_STREAM("batteryStatus call with error");
        battery response = {0, 0, 0, false};
        return response;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "handle_node");
    ros::NodeHandle nh;
    ros::Rate rate(0.2);
    ROS_INFO_STREAM("Donguye geldi.");
    while(ros::ok()){
        position pos = {0,0,2,false};
        moveRelative(pos,nh);
        rate.sleep();
    }
}