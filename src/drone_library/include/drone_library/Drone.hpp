#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


class Drone{
    public:
        Drone(ros::NodeHandle nh);
        
        void move_relative(long double x, long double y, long double z, long double t);

        //Other than the other move function, this fuction takes the exact coordinates to move
        void move_global(long double x, long double y, long double z, long double t);

        //Here x,y,z gives us the center of the circle, r is radius, t is the same as the t in move function.
        void circular_move(long double x, long double y, long double z, long double t, long double r);
        
        bool is_armed();
        
        void arm();

        // Lands exactly to z=0 position
        void land();

        // Lands a different position
        void land(long double x, long double y, long double t);


    private:
        long double _posx, _posy, _posz; //These positions are likely to be changed by another data type
        ros::NodeHandle _nh; //?
        mavros_msgs::State _current_state;
        ros::Subscriber _state_sub;
        ros::ServiceClient _arming_client, _set_mode_client;
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
};
