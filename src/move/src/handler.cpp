#include "move/Handler.hpp"
#include <geometry_msgs/PoseStamped.h>

extern geometry_msgs::PoseStamped pose_command;


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
    #ifdef __arm__
    wiringPiSetupGpio();
    pinMode(PUMP_PIN,OUTPUT);
    pinMode(MOTOR_PIN,OUTPUT);

    digitalWrite(PUMP_PIN,LOW);
    digitalWrite(MOTOR_PIN,LOW);
    #endif
}

Drone::~Drone(){
    #ifdef __arm__
    digitalWrite(PUMP_PIN,LOW);
    digitalWrite(MOTOR_PIN,LOW);
    #else

    #endif
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
 * @brief Takeoff method
 * This needs to be called for takeoff.
 * If already flying, it will give error.
 * 
 * @param z The height to takeoff, in meters.
 * @return true: If the vehicle is not flying at the time.
 * @return false: If the vehicle is flying already.
 */
bool Drone::takeoff(float z){
    move::TkoffLandCommand srv;
    srv.request.cmd = true;
    srv.request.altitude = z;
    ros::ServiceClient client = nh.serviceClient<move::TkoffLandCommand>("move/cmd/tkoff_land");
    if(!flying_status){
        bool success = client.call(srv);
        if(success){
            flying_status = true;
            ROS_INFO_STREAM("Takeoff call with success");
            this->moveRelative({0,0,z,false});
            //pose_command.pose.position.z = z;
            return true;
        }else{
            ROS_ERROR_STREAM("Takeoff call with error");
            
        }
    }
    return false;
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
    move::TkoffLandCommand srv;
    srv.request.cmd = false;
    ros::ServiceClient client = nh.serviceClient<move::TkoffLandCommand>("move/cmd/tkoff_land");

    if(flying_status){
        bool success = client.call(srv);
        if(success){
            flying_status = false;
            ROS_INFO_STREAM("Land call with success");
            return true;
        }else{
            ROS_ERROR_STREAM("Land call with error");
            return false;
        }
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

bool Drone::pumpWrite(int state){
    #ifdef __arm__
    digitalWrite(PUMP_PIN,state == 1 ? HIGH : LOW);
    #endif
}

bool Drone::openCapsule(){
    #ifdef __arm__
    digitalWrite(MOTOR_PIN, HIGH);
    ros::Duration(2.0).sleep();//TODO: use a non blocking method
    digitalWrite(MOTOR_PIN, LOW);
    #endif
}
    //drone.moveRelative({0,0,10,false});

int main(int argc, char **argv){
    ros::init(argc, argv, "handler");
    ros::NodeHandle nh;
    ros::Rate rate(0.2);

    position pos = {-10,-30,10,false};

    Drone drone = Drone(nh);
    ROS_INFO("Takeoff");
    drone.disarm();
    drone.arm();
    ros::Duration(5.0).sleep();
    drone.takeoff(10);
    ros::Duration(20.0).sleep();
    ROS_INFO("Goin");
    drone.moveRelative(pos);
    ros::Duration(30.0).sleep();
    ROS_INFO("Landin");
    drone.land();

    while(ros::ok()){

        ros::spinOnce();
        rate.sleep();
    }
}
