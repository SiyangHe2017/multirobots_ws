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
    ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 1000);
    ros::Publisher cmdpub_2= cmdh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 1000);
    ros::Rate r(4);
    int count = 0;
    while(ros::ok()){
        geometry_msgs::Twist twist;
        geometry_msgs::Vector3 linear;
        if(count%6>2){
            linear.x=0.2;
        }else{
            linear.x = 0.5;
        }
        geometry_msgs::Vector3 angular;
        angular.x=0;
        angular.y=0;
        angular.z=0;
        twist.linear=linear;
        twist.angular=angular;
        cmdpub.publish(twist);
        angular.z = 0.2;
        twist.angular=angular;
        cmdpub_2.publish(twist);
	    count++;
	    ROS_INFO("%d",count);
              
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


