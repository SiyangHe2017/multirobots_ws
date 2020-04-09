#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include<unistd.h>

using namespace std;
int main(int argc,char** argv)
{
    ros::init(argc, argv, "cmdveltest");
    ros::NodeHandle cmdh;
    ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 10, true);;
    ros::Rate r(60);
    int count = 0;
    while(ros::ok()){
        geometry_msgs::Twist twist;
        geometry_msgs::Vector3 linear;
        linear.x=0.1;
        linear.y=0;
        linear.z=0;
        geometry_msgs::Vector3 angular;
        angular.x=0;
        angular.y=0;
        //直行
        //angular.z=0;
        //转圈
        angular.z=-0.5;
        twist.linear=linear;
        twist.angular=angular;
	count++;
	ROS_INFO("%d",count);
        cmdpub.publish(twist);
        ros::spinOnce();
       // ros::spinOnce();
        //r.sleep();
        r.sleep();
    }
    return 0;
}


