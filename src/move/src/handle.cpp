#include <ros/ros.h>
#include <move/Pos.h>

void move_relative(long double _x, long double _y, long double _z, long double _t,ros::NodeHandle nh){
    move::Pos::Response res;
    move::Pos::Request req;
    ros::ServiceClient client = nh.serviceClient<move::Pos>("Pos_global");
    req.x = _x;
    req.y = _y;
    req.z = _z;
    req.t = _t;
    auto succ = client.call(req, res);
    if(succ){
        ROS_INFO_STREAM("Succesfull");
    }
    else{
        ROS_INFO_STREAM("Unsuccesfull");
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
        rate.sleep();
    }
}