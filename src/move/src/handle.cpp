#include <ros/ros.h>
#include <move/Pos.h>
#include <move/Battery.h>

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
        move_relative(2,2,2,0,nh);
        ROS_INFO_STREAM("Request sent to function.");
        std::cout << battery_status(nh).remaining << std::endl; 
        rate.sleep();
    }
}