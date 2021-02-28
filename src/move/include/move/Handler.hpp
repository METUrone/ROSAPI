#ifndef _MOVE_HANDLER_HPP_
#define _MOVE_HANDLER_HPP_
#include <ros/ros.h>

#include <move/Position.h>
#include <move/PositionCommand.h>
#include <move/Battery.h>
#include <move/Camera.h>

typedef struct{
    std::vector<unsigned char> frame;
    bool success;
} frame;



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
        frame camera();

        bool moveGlobal(position pos);
        bool moveRelative(position pos);
        bool takeoff(float z);
        bool land();
        // bool arm(); Absolutely should add this function
        // bool disarm(); Absolutely should add this function

    private:
        battery last_battery_status; // Stores the last known battery status, will be returned if there is a problem in service call. 
        position last_position_info; // Stores the last known position info, will be returned if there is a problem in service call.
        frame last_frame; // Stores the last known frame, will be returned if there is a problem in service call.
        ros::NodeHandle nh; // Is used for service calls.
        bool flying_status; // Tracks the flying status.


};




#endif