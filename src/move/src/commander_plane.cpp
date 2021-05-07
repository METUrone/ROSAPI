#include "move/commander_plane.hpp"

mavros_msgs::State UAV::current_state = {};
sensor_msgs::BatteryState UAV::battery = {};
geometry_msgs::PoseStamped UAV::pose = {};

UAV::UAV(ros::NodeHandle n){

    this->client_takeoff = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");

    this->client_land = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    
    this->client_set_mode = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    this->client_arm = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    this->client_param_get = n.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");

    this->client_param_set = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");

    this->WP_sub = n.subscribe("mavros/mission/reached", 10, UAV::WP_callback);
    
    this->state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, UAV::state_tracker);

    this->battery_status_sub = n.subscribe<sensor_msgs::BatteryState>("mavros/battery",10, UAV::battery_tracker);
    
    this->position_sub = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10, UAV::pose_tracker);

    this->rel_pos_cmd_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    this->server_battery = n.advertiseService("battery_status",&UAV::service_get_battery);
    
    this->server_global_move_command = n.advertiseService("position/command_global",&UAV::service_command_global_position);
    
    this->server_relative_move_command = n.advertiseService("position/command_relative",&UAV::service_command_relative_position);
    
    this->server_position = n.advertiseService("position/position",&UAV::service_get_position);
}

geometry_msgs::Point UAV::getPosition(void)
{
    return UAV::pose.pose.position;
}

sensor_msgs::BatteryState UAV::getBattery(void)
{
    return UAV::battery;
}

mavros_msgs::State UAV::getState(void)
{
    return UAV::current_state;
}

bool UAV::setTakeoff(float altitude)
{
    /* CommandTOL
    float32 min_pitch # used by takeoff
    float32 yaw
    float32 latitude
    float32 longitude
    float32 altitude
    ---
    bool success
    uint8 result
    */
    mavros_msgs::CommandTOL srv;
    srv.request.altitude = altitude;
    if (this->client_takeoff.call(srv))
    {
        ROS_INFO("Takeoff set to %f m altitude",altitude);
        return true;
    }else{
        ROS_ERROR("Failed to call service takeoff!");
    }
    return false;
}

bool UAV::setLand(float altitude)
{
    /* CommandTOL
    float32 min_pitch # used by takeoff
    float32 yaw
    float32 latitude
    float32 longitude
    float32 altitude
    ---
    bool success
    uint8 result
    */
    mavros_msgs::CommandTOL srv;
    srv.request.altitude = altitude;
    if (this->client_land.call(srv))
    {
        ROS_INFO("Landing set to %f m altitude",altitude);
        return true;
    }else{
        ROS_ERROR("Failed to call service land!");
    }
    return false;
}

bool UAV::set_mode(const char* mode)
{
    /* SetMode
    # basic modes from MAV_MODE
    uint8 MAV_MODE_PREFLIGHT = 0
    uint8 MAV_MODE_STABILIZE_DISARMED = 80
    uint8 MAV_MODE_STABILIZE_ARMED = 208
    uint8 MAV_MODE_MANUAL_DISARMED = 64
    uint8 MAV_MODE_MANUAL_ARMED = 192
    uint8 MAV_MODE_GUIDED_DISARMED = 88
    uint8 MAV_MODE_GUIDED_ARMED = 216
    uint8 MAV_MODE_AUTO_DISARMED = 92
    uint8 MAV_MODE_AUTO_ARMED = 220
    uint8 MAV_MODE_TEST_DISARMED = 66
    uint8 MAV_MODE_TEST_ARMED = 194

    uint8 base_mode # filled by MAV_MODE enum value or 0 if custom_mode != ''
    string custom_mode # string mode representation or integer
    ---
    bool mode_sent # Mode known/parsed correctly and SET_MODE are sent
    */
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;
    if (this->client_set_mode.call(srv))
    {
        if(srv.response.mode_sent){
            ROS_INFO("%s mode set!",mode);
            return true;
        }else{
            ROS_WARN("SET_MODE Failed!");
        }
    }else{
        ROS_ERROR("Failed to call service set_mode!");
    }
    return false;
}

bool UAV::arm(bool state=true)
{
    /* CommandBool
    bool value
    ---
    bool success
    uint8 result
    */
    mavros_msgs::CommandBool srv;
    srv.request.value = state;
    if (this->client_arm.call(srv))
    {
        if(srv.response.success){
            ROS_INFO("Vehicle armed!");
            return true;
        }else{
            ROS_WARN("Failed to arm/disarm!");
        }
    }else{
        ROS_ERROR("Failed to call service arm!");
    }
    return false;
}

bool UAV::parameterSet(const char* param_id,int integer=0,float real=0.0){
    /* ParamSet
    string param_id
    mavros_msgs/ParamValue value
    ---
    bool success
    mavros_msgs/ParamValue value
    */
    mavros_msgs::ParamSet srv;
    srv.request.param_id = param_id;
    srv.request.value.integer = integer;
    srv.request.value.real = real;
    if (this->client_param_set.call(srv))
    {
        if(srv.response.success){
            ROS_INFO("Set %s parameter to (%d, %f)",param_id,integer,real);
            return true;
        }else{
            ROS_WARN("Failed to set %s parameter!",param_id);
        }
    }else{
        ROS_ERROR("Failed to call service param set!");
    }
    return false;
}

bool UAV::parameterGet(const char* param_id,mavros_msgs::ParamValue& response){
    /* ParamGet
    string param_id
    ---
    bool success
    ParamValue value
    */
    mavros_msgs::ParamGet srv;
    srv.request.param_id = param_id;
    if (this->client_param_get.call(srv))
    {
        if(srv.response.success){
            response = srv.response.value;
            ROS_INFO("%s parameter: (%d, %f)",param_id,response.integer,response.real);
            return true;
        }else{
            ROS_WARN("Failed to obtain the %s parameter!",param_id);
        }
    }else{
        ROS_ERROR("Failed to call service param get!");
    }
    return false;
}

bool UAV::get_failsafe(bool& DLL,bool& RCL)
{
    /* ParamGet
    string param_id
    ---
    bool success
    ParamValue value
    */
    mavros_msgs::ParamValue resp;
    if (this->parameterGet("NAV_DLL_ACT",resp))
    {
        DLL = resp.integer;
        ROS_INFO("DL Failsafe: %s", DLL ? "true" : "false");
    }else{
        ROS_WARN("Failed to obtain the DLL parameter!");
    }
    if (this->parameterGet("NAV_RCL_ACT",resp))
    {
        RCL = resp.integer;
        ROS_INFO("RC Failsafe: %s", RCL ? "true" : "false");
        return true;
    }else{
        ROS_WARN("Failed to obtain the RCL parameter!");
    }
    return false;
}

bool UAV::set_failsafe(bool DLL=false,bool RCL=false)
{
    /* ParamSet
    string param_id
    mavros_msgs/ParamValue value
    ---
    bool success
    mavros_msgs/ParamValue value
    */
    if (this->parameterSet("NAV_DLL_ACT",DLL,DLL))
    {
        ROS_INFO("Set DL Failsafe to %s", DLL ? "true" : "false");
    }else{
        ROS_WARN("Failed to set DLL parameter!");
    }
    if (this->parameterSet("NAV_RCL_ACT",RCL,RCL))
    {
        ROS_INFO("Set RC Failsafe to %s", RCL ? "true" : "false");
        return true;
    }else{
        ROS_WARN("Failed to set RCL parameter!");
    }
    return false;
}

void UAV::WP_callback(const mavros_msgs::WaypointReached::ConstPtr& msg)
{
    /* WaypointReached
    std_msgs/Header header
    uint16 wp_seq              //index number of reached waypoint
    */
    static int last_id = -1;
    static unsigned long start_sec = 0;//msg->header.stamp.sec;

    if(last_id != (int) msg->wp_seq){
        last_id = (int) msg->wp_seq;
        unsigned long delta_t = msg->header.stamp.sec - start_sec;
        ROS_INFO("Reached # %d WP! Elapsed time: %ld", last_id, delta_t);
    }
}

// Saves the state of the drone
void UAV::state_tracker(const mavros_msgs::State::ConstPtr& _current_state){
    /* State
    std_msgs/Header header
    bool connected
    bool armed
    bool guided
    bool manual_input
    string mode
    uint8 system_status
    */
    UAV::current_state = *_current_state;
}

// Saves the battery state of the drone
void UAV::battery_tracker(const sensor_msgs::BatteryState::ConstPtr& _battery){
    /* BatteryState
    std_msgs/Header  header
    float32 voltage          # Voltage in Volts (Mandatory)
    float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
    float32 current          # Negative when discharging (A)  (If unmeasured NaN)
    float32 charge           # Current charge in Ah  (If unmeasured NaN)
    float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
    float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
    float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
    uint8   power_supply_status     # The charging status as reported. Values defined above
    uint8   power_supply_health     # The battery health metric. Values defined above
    uint8   power_supply_technology # The battery chemistry. Values defined above
    bool    present          # True if the battery is present

    float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                            # If individual voltages unknown but number of cells known set each to NaN
    float32[] cell_temperature  # An array of individual cell temperatures for each cell in the pack
                                # If individual temperatures unknown but number of cells known set each to NaN
    string location          # The location into which the battery is inserted. (slot number or plug)
    string serial_number     # The best approximation of the battery serial number
    */
    UAV::battery = *_battery;
}

// Saves the pose of the drone
void UAV::pose_tracker(const geometry_msgs::PoseStamped::ConstPtr& _pose){
    /* PoseStamped
    std_msgs/Header header
    geometry_msgs/Pose pose
    /* Pose
    geometry_msgs/Point position              // x y z
    geometry_msgs/Quaternion orientation      // x y z w
    */
    UAV::pose = *_pose;
}

// Responses the voltage, current and remaining percentage of battery when the service called.
bool UAV::service_get_battery(move::Battery::Request &req,move::Battery::Response &res)
{
    ROS_INFO_STREAM("Battery service is called");
    res.voltage = UAV::battery.voltage;
    res.current = UAV::battery.current;
    res.remaining = UAV::battery.percentage;
    return true;
}

// Responses the position of the drone, x,y,z when the service is called.
bool UAV::service_get_position(move::Position::Request &req,move::Position::Response &res)
{
    res.x = UAV::pose.pose.position.x;
    res.y = UAV::pose.pose.position.y;
    res.z = UAV::pose.pose.position.z;
    return true;
}

geometry_msgs::PoseStamped pose_command;

// When the service called,
// the requested x,y,z and t is put to the pose_command.
bool UAV::service_command_global_position(move::PositionCommand::Request &req,move::PositionCommand::Response &res)
{
    ROS_INFO_STREAM("Move service is called");
    pose_command.pose.position.x = req.x;
    pose_command.pose.position.y = req.y;
    pose_command.pose.position.z = req.z;
    return true;
}

// When the service called,
// the requested x,y,z and t is added to the pose_command.
bool UAV::service_command_relative_position(move::PositionCommand::Request &req,move::PositionCommand::Response &res)
{
    ROS_INFO_STREAM("Move service is called");
    pose_command.pose.position.x += req.x;
    pose_command.pose.position.y += req.y;
    pose_command.pose.position.z += req.z;
    return true;
}

bool UAV::takeoff(float altitude,bool blocking=true){
    this->parameterSet("RWTO_TKOFF",1);
    this->setTakeoff(altitude);
    this->arm();
    this->set_mode("AUTO.TAKEOFF");
    ros::Rate rate(20.0);
    while(this->getState().mode == "AUTO.TAKEOFF" && ros::ok() && blocking){
        ros::spinOnce();
        rate.sleep();
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_plane");
    ros::Time::init();
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    UAV plane(nh);

    // wait for FCU connection
    while(ros::ok() && !plane.getState().connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Read Datalink and RC failsafe STATUS. Remove if present (for SITL)!
    bool DL_failsafe,RC_failsafe;
    plane.get_failsafe(DL_failsafe,RC_failsafe);
    if (DL_failsafe || RC_failsafe) plane.set_failsafe(false,false);

    //send a few setpoints before starting
    for(int i = 40; ros::ok() && i > 0; --i){ // The number of loops may be decreased probably.
        plane.rel_pos_cmd_pub.publish(pose_command);
        ros::spinOnce();
        rate.sleep();
    }
    
    pose_command.pose.position.x = 30;
    pose_command.pose.position.y = 30;
    pose_command.pose.position.z = 6;
    geometry_msgs::Point pos;
    ros::Time last_request_time = ros::Time::now();
    while(ros::ok()){

        if( (plane.getState().mode != "OFFBOARD") && (ros::Time::now()-last_request_time > ros::Duration(3.0)) ){
            if( plane.set_mode("OFFBOARD")){
                //ROS_INFO_STREAM("Offboard enabled");
            }
            last_request_time = ros::Time::now();
        }
        else{
            if( !plane.getState().armed && (ros::Time::now() - last_request_time > ros::Duration(3.0)) ){
                plane.takeoff(pose_command.pose.position.z);
                last_request_time = ros::Time::now();
            }
        }

        plane.rel_pos_cmd_pub.publish(pose_command);
        
        pos = plane.getPosition();
        ROS_INFO("Pos: %f,%f,%f",pos.x,pos.y,pos.z);

        ros::spinOnce();
        rate.sleep();
    }
}