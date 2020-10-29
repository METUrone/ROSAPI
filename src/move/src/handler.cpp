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

class Drone{
    public:
        Drone(ros::NodeHandle _nh);

        position takePositionInfo();
        battery batteryStatus();

        bool moveGlobal(position pos);
        bool moveRelative(position pos);
        bool takeoff(double z);

    private:
        battery last_battery_status = {0};
        position last_position_info = {0};
        ros::NodeHandle nh;
        bool flying_status;


};


Drone::Drone(ros::NodeHandle _nh){
    nh = _nh;
    flying_status = false;
}

bool Drone::takeoff(double z){
    if(flying_status == true){
        ROS_ERROR_STREAM("takeoff called but already flying!");
        return false;
    }
    else{
        moveRelative({0,0,z,true});
        // Here may need to sleep a while, then control if movement is done or still running.
        // But for now I do not write such a thing
        flying_status = true;
        return true;
    }
}

position Drone::takePositionInfo(){
    move::Position::Response res;
    move::Position::Response req;
    ros::ServiceClient client = nh.serviceClient<move::Position>("position/position");

    bool success = client.call(req,res);
    if(success){
        ROS_INFO_STREAM("TakePositionInfo call with success");
        position response = {res.x, res.y, res.z, true};
        last_position_info = {res.x, res.y ,res.z ,false};
        return response;
    }
    else{
        ROS_INFO_STREAM("TakePositionInfo call with error");
        position response = last_position_info;
        return response;
    }
}

bool Drone::moveGlobal(position pos){
    if(!flying_status){
        return false;
    }

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

bool Drone::moveRelative(position pos){
    if(!flying_status){
        return false;
    }

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

battery Drone::batteryStatus(){
    move::Battery::Response res;
    move::Battery::Request req;
    ros::ServiceClient client = nh.serviceClient<move::Battery>("battery_status");

    bool success = client.call(req,res);
    if(success){
        ROS_INFO_STREAM("batteryStatus call with success");
        battery response = {res.voltage, res.current, res.remaining, true};
        last_battery_status = {res.voltage, res.current, res.remaining, false};
        return response;
    }
    else{
        ROS_INFO_STREAM("batteryStatus call with error");
        battery response = last_battery_status;
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
        //moveRelative(pos,nh);
        rate.sleep();
    }
}