#include <ros/ros.h>
#include <move/Position.h>
#include <move/PositionCommand.h>
#include <move/Battery.h>

/**
 * @brief Position struct to send and receive positions to drone.
 * Be careful that x,y and z's type is float. Type conversion may be a big problem sometimes.
 * The success variable is not used for setting. When called a method to take data, it will be setted to false when there is a problem.
 */
typedef struct{
    float x;
    float y;
    float z;
    bool success;
} position;

/**
 * @brief Battery struct to receive the battery's status.
 * Be careful that x,y and z's type is float. Type conversion may be a big problem sometimes.
 * The success variable is not used for setting. When called a method to take data, it will be setted to false when there is a problem. 
 */
typedef struct{
    float voltage;
    float current;
    float remaining;
    bool success;
} battery;
/**
 * @brief Drone class to control the drone
 * The class will be used always when controlling the drone with this API.
 * It has basic functions, and is designed to send the commands, then give the control to you immediately. Meaning that you have full control of the drone.
 * 
 */
class Drone{
    public:
        Drone(ros::NodeHandle _nh);

        position takePositionInfo();
        battery batteryStatus();

        bool moveGlobal(position pos);
        bool moveRelative(position pos);
        bool takeoff(float z);

    private:
        battery last_battery_status; // Stores the last known battery status, will be returned if there is a problem in service call. 
        position last_position_info; // Stores the last known position info, will be returned if there is a problem in service call.
        ros::NodeHandle nh; // Is used for service calls.
        bool flying_status; // Tracks the flying status.


};

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
        moveRelative({0,0,z,true});
        // Here may need to sleep a while, then control if movement is done or still running.
        // But for now I do not write such a thing
        flying_status = true;
        return true;
    }
}
/**
 * @brief Returns the GPS position of the vehicle.
 * If somehow the service call is not succeded, returns the latest position known, with position.success equals to false.
 * 
 * @return position struct. x,y and z is the coordinates, and success is for service call succeded or not.
 */
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