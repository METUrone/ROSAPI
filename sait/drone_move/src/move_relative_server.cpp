#include <ros/ros.h>
#include <msgs/move_relative_msg.h>//while giden mesaj
#include <msgs/move_relative_server.h> //server msg

void send_msg(
	//move relatif info topicine msg gönderildi msg-> x ,y ,z ,t
	//msgs/move_relative_msg.h dan değiştirlebilir
	ros::NodeHandle nh;
	msgs::move_relative_server::Request &req,
	msgs::move_relative_server::Response &resp){
	ros::Publisher pub=nh.advertise<msgs::move_relative_msg>("move_relative_info",1000);
	msgs::move_relative_server msg;
	msg.x=req.x;
	msg.y=req.y;
	msg.z=req.z;
	msg.t=req.t;
	pub.publish(msg);
	ros::spinOnce();
	ROS_INFO_STREAM("sending msg from dronemove to while");
int main(int argc,char **argv){
	ros :: init (argc,argv,"move_relative_server");
	ros :: NodeHandle nh;
	ros :: ServiceSerer server =nh.advertiseService("it_is_about_sending_a_message",&send_msg);
	

