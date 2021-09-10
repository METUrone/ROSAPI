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
        ROS_INFO("TakePositionInfo call with success");
        position response = {res.x, res.y, res.z, true};
        last_position_info = {res.x, res.y ,res.z ,false};
        return response;
    }
    else{
        ROS_INFO("TakePositionInfo call with error");
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
        ROS_INFO("moveGlobal call with success");
        return true;
    }
    else{
        ROS_INFO("moveGlobal call with error");
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
        ROS_INFO("moveRelative call with success");
        return true;
    }
    else{
        ROS_INFO("moveRelative call with error");
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
            ROS_INFO("Takeoff call with success");
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
            ROS_INFO("Land call with success");
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
        ROS_INFO("batteryStatus call with success");
        battery response = {res.voltage, res.current, res.remaining, true};
        last_battery_status = {res.voltage, res.current, res.remaining, false};
        return response;
    }
    else{
        ROS_INFO("batteryStatus call with error");
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

bool Drone::wherePool(double& _posx, double& _posy, double& _posx2, double& _posy2){
    ros::ServiceClient get_pool = nh.serviceClient<move::Mesafe>("mesafe");

    move::Mesafe srv;
    if(get_pool.call(srv)){
        if((srv.response.posx > 9e4) || (srv.response.posy > 9e4) || (srv.response.posx2 > 9e4) || (srv.response.posy2 > 9e4)){
            ROS_INFO("No Object Detected");
        }else{
            ROS_INFO("Object detected at x:%.2f   y:%.2f    x2:%.2f    y2:%.2f",srv.response.posx,srv.response.posy,srv.response.posx2,srv.response.posy2);
            _posx = srv.response.posx;
            _posy = srv.response.posy;
            _posx2 = srv.response.posx2;
            _posy2 = srv.response.posy2;
            return true;
        }
    }else{
        ROS_WARN("Couldn't call cv_ws");
    }
    return false;
}
void mission1() {
    // ################################
    // # CODE FOR MISSION 1 GOES HERE #
    // ################################
    ros::NodeHandle nh;
    ros::Rate rate(0.2);

    position pos = {10,30,0,false};

    Drone drone = Drone(nh);
    ROS_INFO("Takeoff");
    drone.disarm();
    drone.arm();
    ros::Duration(5.0).sleep();
    ros::Rate little_sleep(5);
    int height = 10;
    drone.takeoff(height);
	while(1) {
        ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().z >= height-1) && (drone.takePositionInfo().z <= height+1)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
	//circle_polar(drone, {-9,0,0,0}, 128, 4);
    //ros::Duration(2.0).sleep();
    //do_task1();
    ROS_INFO("doing start");
    int right_size = 5;
      drone.moveRelative({right_size,0,0,true});
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().x >= right_size-0.5) && (drone.takePositionInfo().x <= right_size+0.5)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
	ROS_INFO("move 1 complete!!");
      //ros::Duration(3.0).sleep();
      int upper_size = 3;
      drone.moveRelative({1,-upper_size,0,true});
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().y >= -upper_size-1) && (drone.takePositionInfo().y <= -upper_size+1)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      ROS_INFO("move 2 complete!!");
      //ros::Duration(3.0).sleep();
      drone.moveRelative({-1,-upper_size,0,true});
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().y >= ((-2*upper_size)-1)) && (drone.takePositionInfo().y <= ((-2*upper_size)+1))) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      //ros::Duration(3.0).sleep();
      drone.moveRelative({-right_size,0,0,true});
      ROS_INFO("move 3 complete!!");
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().x >= -0.6) && (drone.takePositionInfo().x <= 0.6)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      //ros::Duration(3.0).sleep();
      drone.moveRelative({-1,-2,0,true});
      ROS_INFO("move 4 complete!!");
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().y >= -9) && (drone.takePositionInfo().y <= -7)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      //ros::Duration(3.0).sleep();
      drone.moveRelative({2,0,0,true});
      ROS_INFO("move 5 complete!!");
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().x >= 0.5) && (drone.takePositionInfo().x <= 1.5)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      //ros::Duration(3.0).sleep();
      drone.moveRelative({-1,2,0,true});
      ROS_INFO("move 6 complete!!");
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().x >= -0.5) && (drone.takePositionInfo().x <= 0.5)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      //ros::Duration(3.0).sleep();
      drone.moveRelative({-5,0,0,true});
      ROS_INFO("move 7 complete!!");
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().x >= -6) && (drone.takePositionInfo().x <= -4)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      //ros::Duration(3.0).sleep();
      drone.moveRelative({-1,3,0,true});
      ROS_INFO("move 8 complete!!");
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().y >= -3.5) && (drone.takePositionInfo().y <= -2.5)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      //ros::Duration(3.0).sleep();
      drone.moveRelative({1,3,0,true});
      ROS_INFO("move 9 complete!!");
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().y >= -0.5) && (drone.takePositionInfo().y <= 0.5)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }
      //ros::Duration(3.0).sleep();
      drone.moveRelative({5,0,0,true});
      ROS_INFO("To origin we go !!");
      while(1) {
          ROS_INFO(" x:%f | y: %f | z:%f", drone.takePositionInfo().x, drone.takePositionInfo().y, drone.takePositionInfo().z);
         if ((drone.takePositionInfo().x >= -1) && (drone.takePositionInfo().x <= 1)) {
            break;
         }
         little_sleep.sleep();
         continue;
      }

    drone.land();
    ros::Duration(3.0).sleep();
}

void mission2() {
    // ################################
    // # CODE FOR MISSION 2 GOES HERE #
    // ################################
    ros::NodeHandle nh;
    ros::Rate rate(0.2);

    position pos = {10,30,0,false};

    Drone drone = Drone(nh);
    ROS_INFO("Takeoff");
    drone.disarm();
    drone.arm();
    ros::Duration(5.0).sleep();
    drone.takeoff(15);
    
    float x_location_of_water_grab = 35; //This information will later be obtained by computer vision.
    float y_location_of_water_grab = -25; //This information will later be obtained by computer vision.
    float x_location_of_water_drop = 15; //This information will later be obtained by computer vision.
    float y_location_of_water_drop = -25; //This information will later be obtained by computer vision.
    
    while(1) {
         if ((drone.takePositionInfo().z >= 14.5) && (drone.takePositionInfo().z <= 16)) {
            break;
         }
         continue;
      }
    ROS_INFO("doing start");
      
	ROS_INFO("Go forward! ! !");

      drone.moveRelative({40,0,0,true}); // Aslında 50 değil 40 
      while(1) {
         if ((drone.takePositionInfo().x >= 37.5) && (drone.takePositionInfo().x <= 42.5)) {
            break;
         }
         continue;
      }
	ROS_INFO("move 1 complete!!");
      //ros::Duration(3.0).sleep();
      drone.moveRelative({5,-15,0,true}); //Aslında 18 değil 15
      while(1) {
         if ((drone.takePositionInfo().y >= -16) && (drone.takePositionInfo().y <= -14)) {
            break;
         }
         continue;
      }
      ROS_INFO("move 2 complete!!");
      //ros::Duration(3.0).sleep();
      drone.moveRelative({-5,-15,0,true}); //Aslında 18 değil 15
      while(1) {
         if ((drone.takePositionInfo().y >= -31) && (drone.takePositionInfo().y <= -29)) {
            break;
         }
         continue;
      }
    drone.moveRelative({-80,0,0,true}); // 100 değil 80
     while(1) {
         if ((drone.takePositionInfo().x >= -44) && (drone.takePositionInfo().x <= -36)) {
            break;
         }
         continue;
      }
    
ROS_INFO("turning (old problem spot)!!");
    drone.moveRelative({-5,15,0,true}); // 18 değil 15
      
      while(1) {
         if ((drone.takePositionInfo().y >= -17) && (drone.takePositionInfo().y <= -13)) {
            break;
         }
         continue;
      }
      ROS_INFO("turning!!");
      drone.moveRelative({5,15,0,true}); // 18 değil 15
      
      while(1) {
         if ((drone.takePositionInfo().y >= -1) && (drone.takePositionInfo().y <= 1)) {
            break;
         }
         continue;
      }
    ROS_INFO("moving!!");
    drone.moveRelative({80,-10,0,true}); // 100 değil 80
    while(1) {
         if ((drone.takePositionInfo().x >= 39) && (drone.takePositionInfo().x <= 40.5)) {
            break;
         }
         continue;
      }
    ROS_INFO("turning again!!");
      //ros::Duration(3.0).sleep();
      drone.moveRelative({5,-5,0,true}); // 7 değil 5
      while(1) {
         if ((drone.takePositionInfo().x >= 44) && (drone.takePositionInfo().x <= 46)) {
            break;
         }
         continue;
      }
      ROS_INFO("turning!!");
      //ros::Duration(3.0).sleep();
      drone.moveRelative({-5,-5,0,true}); // 7 değil 5
      while(1) {
         if ((drone.takePositionInfo().y >= -21) && (drone.takePositionInfo().y <= -19)) {
            break;
         }
         continue;
      }
    drone.moveRelative({x_location_of_water_grab-40,y_location_of_water_grab+20,-12,true});
    while(1) {
         if ((drone.takePositionInfo().x >= x_location_of_water_grab-1) && (drone.takePositionInfo().x <= x_location_of_water_grab+1) && (drone.takePositionInfo().z >= 2) && (drone.takePositionInfo().z <= 4)) {
            break;
         }
         continue;
      }
    ROS_INFO("Grabbing water");
    drone.moveRelative({0,0,12,true});
    while(1) {
         if ((drone.takePositionInfo().z >= 14.5) && (drone.takePositionInfo().z <= 16)) {
            break;
         }
         continue;
      }
    ROS_INFO("Let us bomb now.");
    drone.moveRelative({x_location_of_water_drop - x_location_of_water_grab,y_location_of_water_drop-y_location_of_water_grab,-12,true});
    while(1) {
         if ((drone.takePositionInfo().x >= x_location_of_water_drop-1) && (drone.takePositionInfo().x <= x_location_of_water_drop+1) && (drone.takePositionInfo().z >= 2) && (drone.takePositionInfo().z <= 4)) {
            break;
         }
         continue;
      }
    ROS_INFO("Found the drop zone");
    
    ROS_INFO("Sending the bomb");
    
    ROS_INFO("Let us continue");
    drone.moveRelative({-40-x_location_of_water_drop,-20-y_location_of_water_drop,12,true});
    while(1) {
	if ((drone.takePositionInfo().x >= -42) && (drone.takePositionInfo().x <= -38)) {
		break;	
	}
	continue;	
	}
    ROS_INFO("turning!!");
    drone.moveRelative({-5,5,0,true}); // 7 değil 5
      
      while(1) {
         if ((drone.takePositionInfo().y >= -16) && (drone.takePositionInfo().y <= -13.5)) {
            break;
         }
         continue;
      }
      ROS_INFO("turning!!");
      drone.moveRelative({5,5,0,true}); // 7 değil 5
      
      while(1) {
         if ((drone.takePositionInfo().y >= -12) && (drone.takePositionInfo().y <= -8)) {
            break;
         }
         continue;
      }
    drone.moveRelative({40,10,0,true}); //
      
      while(1) {
         if ((drone.takePositionInfo().x >= -1) && (drone.takePositionInfo().x <= 5)) {
            break;
         }
         continue;
      }
ROS_INFO("Finally landed.!!");
    drone.land();
while(1) {
         if ((drone.takePositionInfo().z >= -1) && (drone.takePositionInfo().z <= 0.5)) {
            break;
         }
         continue;
      }

    ros::Duration(3.0).sleep();

}

int main(int argc, char **argv){
    ros::init(argc, argv, "handler");
    
    
    //MISSION CODES COULD BE INVOKED AFTER THIS LINE
    mission1();




    /*
    drone.takeoff(10);
    ros::Duration(20.0).sleep();
    ROS_INFO("Goin");
    drone.moveRelative(pos);
    ros::Duration(20.0).sleep();
    ROS_INFO("Landin");
    drone.land();
    */
    ros::Rate newRate(0.2);
    while(ros::ok()){

        ros::spinOnce();
        newRate.sleep();
    }
    
}
