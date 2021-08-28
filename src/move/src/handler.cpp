#include "move/Handler.hpp"

/**
 * @brief Construct a new Drone::Drone object
 * 
 * @param _nh This nodeHandle will be used in all serviceClients.
 */
Drone::Drone(ros::NodeHandle _nh){
    nh = _nh;
    last_battery_status = {0,0,0,false};
    last_position_info = {0,0,0,false};
    flying_status = false;
}

bool Drone::arm(){
    move::ArmDisarmCommand::Request req;
    move::ArmDisarmCommand::Response res;
    ros::ServiceClient client = nh.serviceClient<move::ArmDisarmCommand>("arm_disarm");
    req.cmd = true;
    return client.call(req,res);
}

bool Drone::disarm(){
    move::ArmDisarmCommand::Request req;
    move::ArmDisarmCommand::Response res;
    ros::ServiceClient client = nh.serviceClient<move::ArmDisarmCommand>("arm_disarm");
    req.cmd = false;
    return client.call(req,res);
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
bool Drone::takeoff(float z){
    if(flying_status == true){
        ROS_ERROR_STREAM("takeoff called but already flying!");
        return false;
    }
    else{
        flying_status = true;
        moveRelative({0,0,z,true});
        // Here may need to sleep a while, then control if movement is done or still running.
        // But for now I do not write such a thing
        return true;
    }
}


/**
 * @brief Lands the drone.
 * If takePositionInfo succeded, sends the commands to land. If not, it do not send command, and returns false.
 * Be careful that if there is a problem in landing, you have the full control to land. Watch the return.
 * If things go wrong, the last thing you may do is closing commander. The drone will land automatically.
 * 
 * 
 * @return true if command is sent successfully.
 * @return false if not already flying or command couldn't be sent successfully.
 */
bool Drone::land(){
    // In future, we may use landing flight mode to land, but not now. 
    if(!flying_status){
        return false;
    }
    position pos = takePositionInfo();
    if(pos.success){
        moveRelative({0,0,-pos.z,false});
        // To do
        // Here may sleep a while, then check if landing is correctly done or not.
        flying_status = false;
        return true;
    }
    else{
        return false;
    }
}


/**
 * @brief Returns the GPS position of the vehicle.
 * If somehow the service call is not succeded, returns the latest position known, with position.success equals to false.
 * 
 * @return position struct. x,y and z is the coordinates, and success is for service call succeded or not.
 */
position Drone::takePositionInfo(){
    move::Position::Request req;
    move::Position::Response res;
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
        return last_position_info;
    }
}


/**
 * @brief Move method with global coordinates.
 * Be careful with this method. If you give (0,0,0), the vehicle will go to 'Null Island', and land in there. You may not want your drone to be wet.
 * It is advisable to use moveGlobal after taking the coordinates with takePositionInfo method. Should not be called before takeoff, the method will give an error.
 * 
 * @param pos takes position to go, x,y, and z. The units are meters .The success parameter do not matter.
 * @return true: If already flying, and the service call is succeded, returns true.
 * @return false: If not in flying status, or there is a problem in service call, returns false.
 */
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


/**
 * @brief Move method with relative coordinates.
 * Moves the drone with the coordinates, the reference point is the drone's itself.
 * Meaning that if called with (1,0,0), the drone will go 1 meter in x axis. Should not be called before takeoff, the method will give an error.
 * 
 * @param pos takes position to go, x,y, and z. The units are meters .The success parameter do not matter.
 * @return true: If already flying, and the service call is succeded, returns true.
 * @return false: If not in flying status, or there is a problem in service call, returns false.
 */
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


/**
 * @brief Returns the battery's status
 * If somehow the service call is not succeded, returns the latest battery status known, with battery.success equals to false.
 * 
 * 
 * @return battery struct. If the call succeded, returns battery.success true, otherwise false.
 */
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
        return last_battery_status;
    }
}


/**
 * @brief Returns a camera frame
 * 
 * @return frame 
 */
frame Drone::camera(){
    move::Camera::Response res;
    move::Camera::Request req;

    ros::ServiceClient client = nh.serviceClient<move::PositionCommand>("camera");

    bool success = client.call(req,res);

    if(success){
        frame image = {res.data, true};
        last_frame = {res.data, false};
        return image;
    }
    else{
        return last_frame;
    }
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "handler");
    ros::NodeHandle nh;
    ros::Rate rate(0.2);
    Drone drone = Drone(nh);
    drone.takeoff(5);
    while(ros::ok()){
        position pos = {0,0,5,false};
        rate.sleep();
    }
}
