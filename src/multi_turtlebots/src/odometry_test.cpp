#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <iostream>

void chatterCallback(const nav_msgs::Odometry::ConstPtr &msg){
	ROS_INFO("%s", msg->header.frame_id.c_str());
	ROS_INFO("%f", msg->twist.twist.linear.x);
	ROS_INFO("%f", msg->twist.twist.angular.y);
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "odometry_test");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/tb3_0/odom", 1, chatterCallback);
	ros::Subscriber sub_2 = nh.subscribe("tb3_1/odom", 1, chatterCallback);
	//ros::Rate loop_rate(4);
	// while(ros::ok()){
		
		//
	// ros::spinOnce();
	ros::Duration(1).sleep();
	ros::spinOnce();
	std::cout << "at the end" << std::endl;
		//loop_rate.sleep();
	// }
	return 0;
}
