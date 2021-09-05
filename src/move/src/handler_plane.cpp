#include "move/Handler_plane.hpp"

/**
 * @brief Construct a new Plane::Plane object
 * 
 * @param _nh This nodeHandle will be used in all serviceClients.
 */
Plane::Plane(ros::NodeHandle _nh){
    nh = _nh;
    last_battery_status = {0,0,0,false};
    last_position_info = {0,0,0,false};
    flying_status = false;
}


/**
 * @brief Takeoff method
 * This needs to be called for takeoff.
 * If already flying, it will give error.
 * 
 * @param z The height to takeoff, in meters.
 * @return true: If the vehicle is not flying at the time.
 * @return false: If the vehicle is flying already.
 */
bool Plane::takeoff(float z){
    move::TkoffLandCommand srv;
    srv.request.cmd = true;
    srv.request.altitude = z;
    ros::ServiceClient client = nh.serviceClient<move::TkoffLandCommand>("move/cmd/tkoff_land");

    bool success = client.call(srv);
    if(success){
        flying_status = true;
        ROS_INFO_STREAM("Takeoff call with success");
        return true;
    }else{
        ROS_INFO_STREAM("Takeoff call with error");
        return false;
    }
}


/**
 * @brief Lands the plane.
 * If takePositionInfo succeded, sends the commands to land. If not, it do not send command, and returns false.
 * Be careful that if there is a problem in landing, you have the full control to land. Watch the return.
 * If things go wrong, the last thing you may do is closing commander. The plane will land automatically.
 * 
 * 
 * @return true if command is sent successfully.
 * @return false if not already flying or command couldn't be sent successfully.
 */
bool Plane::land(float z){
    move::TkoffLandCommand srv;
    srv.request.cmd = false;
    srv.request.altitude = z;
    ros::ServiceClient client = nh.serviceClient<move::TkoffLandCommand>("move/cmd/tkoff_land");

    bool success = client.call(srv);
    if(success){
        flying_status = false;
        ROS_INFO_STREAM("Land call with success");
        return true;
    }else{
        ROS_INFO_STREAM("Land call with error");
        return false;
    }
}

move::State Plane::getState(){
    move::State srv;
    ros::ServiceClient client = nh.serviceClient<move::State>("move/get/state");
    
    bool success = client.call(srv);
    if(success){
        ROS_INFO_STREAM("Arm call with success");
    }else{
        ROS_WARN_STREAM("Could not get the UAV State!");
    }
    return srv;
}

bool Plane::isArmed(){
    return this->getState().response.armed;
}

bool Plane::arm(){
    move::ArmDisarmCommand srv;
    srv.request.cmd = true;
    ros::ServiceClient client = nh.serviceClient<move::ArmDisarmCommand>("move/cmd/arm_disarm");

    bool success = client.call(srv);
    if(success){
        ROS_INFO_STREAM("Arm call with success");
        return true;
    }else{
        ROS_INFO_STREAM("Arm call with error");
        return false;
    }
}

bool Plane::disarm(){
    move::ArmDisarmCommand srv;
    srv.request.cmd = false;
    ros::ServiceClient client = nh.serviceClient<move::ArmDisarmCommand>("move/cmd/arm_disarm");

    bool success = client.call(srv);
    if(success){
        ROS_INFO_STREAM("Arm call with success");
        return true;
    }else{
        ROS_INFO_STREAM("Arm call with error");
        return false;
    }
}

/**
 * @brief Returns the GPS position of the vehicle.
 * If somehow the service call is not succeded, returns the latest position known, with position.success equals to false.
 * 
 * @return position struct. x,y and z is the coordinates, and success is for service call succeded or not.
 */
position Plane::takePositionInfo(){
    move::Position::Response res;
    move::Position::Response req;
    ros::ServiceClient client = nh.serviceClient<move::Position>("move/get/position");

    bool success = client.call(req,res);
    if(success){
        ROS_INFO_STREAM("TakePositionInfo call with success");
        position response = {res.x, res.y, res.z, true};
        last_position_info = {res.x, res.y ,res.z ,false};
        return response;
    }
    else{
        ROS_INFO_STREAM("TakePositionInfo call with error");
        return last_position_info;
    }
}


/**
 * @brief Move method with global coordinates.
 * Be careful with this method. If you give (0,0,0), the vehicle will go to 'Null Island', and land in there. You may not want your plane to be wet.
 * It is advisable to use moveGlobal after taking the coordinates with takePositionInfo method. Should not be called before takeoff, the method will give an error.
 * 
 * @param pos takes position to go, x,y, and z. The units are meters .The success parameter do not matter.
 * @return true: If already flying, and the service call is succeded, returns true.
 * @return false: If not in flying status, or there is a problem in service call, returns false.
 */
bool Plane::moveGlobal(position pos){
    if(!flying_status){
        return false;
    }

    move::PositionCommand::Response res;
    move::PositionCommand::Request req;
    ros::ServiceClient client = nh.serviceClient<move::PositionCommand>("move/cmd/position_global");

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


/**
 * @brief Move method with relative coordinates.
 * Moves the plane with the coordinates, the reference point is the plane's itself.
 * Meaning that if called with (1,0,0), the plane will go 1 meter in x axis. Should not be called before takeoff, the method will give an error.
 * 
 * @param pos takes position to go, x,y, and z. The units are meters .The success parameter do not matter.
 * @return true: If already flying, and the service call is succeded, returns true.
 * @return false: If not in flying status, or there is a problem in service call, returns false.
 */
bool Plane::moveRelative(position pos){
    if(!flying_status){
        return false;
    }

    move::PositionCommand::Response res;
    move::PositionCommand::Request req;
    ros::ServiceClient client = nh.serviceClient<move::PositionCommand>("move/cmd/position_relative");

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


/**
 * @brief Returns the battery's status
 * If somehow the service call is not succeded, returns the latest battery status known, with battery.success equals to false.
 * 
 * 
 * @return battery struct. If the call succeded, returns battery.success true, otherwise false.
 */
battery Plane::batteryStatus(){
    move::Battery::Response res;
    move::Battery::Request req;
    ros::ServiceClient client = nh.serviceClient<move::Battery>("move/get/battery_status");

    bool success = client.call(req,res);

    if(success){
        ROS_INFO_STREAM("batteryStatus call with success");
        battery response = {res.voltage, res.current, res.remaining, true};
        last_battery_status = {res.voltage, res.current, res.remaining, false};
        return response;
    }
    else{
        ROS_INFO_STREAM("batteryStatus call with error");
        return last_battery_status;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "handler_plane");
    ros::NodeHandle nh;
    ros::Rate rate(0.2);
    Plane plane = Plane(nh);
    ROS_INFO("Takeoff");
    plane.takeoff(10.0);
    ROS_INFO("To Origin");
    position pos = {0,0,20,false};
    plane.moveGlobal(pos);
    ros::Duration(10.0).sleep();
    pos = {20,20,30,false};
    plane.moveGlobal(pos);
    ros::Duration(10.0).sleep();
    ROS_INFO("Land");
    plane.land(0);
}
