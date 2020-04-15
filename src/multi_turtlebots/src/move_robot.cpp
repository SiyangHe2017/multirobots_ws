#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif


#include <RVO.h>

#include <algorithm>
#include <math.h>
#include <iostream>

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>

#define ANGULAR_VELOCITY 0.5
#define PI 3.1415926


std::vector<RVO::Vector2> goals;
float tb3_x_position[2];
float tb3_y_position[2];
float tb3_theta[2];
float tb3_linear_velocity[2];
float tb3_angular_velocity[2];
bool tb3_reached[2] = {0};
float k_goal_parameter = 0.5;


void setupScenario(RVO::RVOSimulator* sim) {
  // Specify global time step of the simulation.
  sim->setTimeStep(0.25f);

  // setAgentDefaults (float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity=Vector2())
  // Specify default parameters for agents that are subsequently added.
  sim->setAgentDefaults(8.0f, 10.0f, 8.0f, 8.0f, 0.12f, 0.8f);

  // Add agents, specifying their start position.
  sim->addAgent(RVO::Vector2(-2.0f, -2.0f));
  sim->addAgent(RVO::Vector2(-1.0f, 1.0f));  // Notice here !!

  // Create goals (simulator is unaware of these).
  for (std::size_t i = 0; i < sim->getNumAgents(); ++i) {
    goals.push_back(-sim->getAgentPosition(i));
  }

  // ROS_INFO("%ld",sim->getNumAgents()); // for test sim agent numbers
  // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
  // std::vector<RVO::Vector2> vertices;
  // vertices.push_back(RVO::Vector2(-7.0f, -20.0f));
  // vertices.push_back(RVO::Vector2(7.0f, -20.0f));
  // vertices.push_back(RVO::Vector2(7.0f, 20.0f));
  // vertices.push_back(RVO::Vector2(-7.0f, 20.0f));

  // sim->addObstacle(vertices);

  // Process obstacles so that they are accounted for in the simulation.
  // sim->processObstacles();
  // Obstacles added to the simulation after this function has been called are not accounted for in the simulatio
}

void setPositionToRVO(RVO::RVOSimulator *sim){
  for(std::size_t i=0; i < sim->getNumAgents(); ++i){
    sim->setAgentPosition(i, RVO::Vector2(tb3_x_position[i], tb3_y_position[i]));
  }
}

void setPreferredVelocities(RVO::RVOSimulator* sim) {
  // Set the preferred velocity for each agent.
  for (std::size_t i = 0; i < sim->getNumAgents(); ++i) {
    if (RVO::absSq(goals[i] - sim->getAgentPosition(i)) < k_goal_parameter * sim->getAgentRadius(i) * sim->getAgentRadius(i) ) {
      // Agent is within one radius of its goal, set preferred velocity to zero
      sim->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
    } else {
      // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
      sim->setAgentPrefVelocity(i, 0.15*RVO::normalize(goals[i] - sim->getAgentPosition(i)));
    }
  }
}

void setVelocityToTurtlebot3(RVO::RVOSimulator* sim){
  for (std::size_t i = 0; i < sim->getNumAgents(); ++i) {
    // linear velocity stands for velocity in x
    tb3_linear_velocity[i] = RVO::abs(sim->getAgentVelocity(i));

    // angular velocity is determined by orientation and velocity tang
    float temp_orientation = tb3_theta[i];
    // float temp_velocity_tan = atan2(tb3_y_position[i], tb3_x_position[i]); // problem here !!
    float temp_velocity_tan = atan2(sim->getAgentVelocity(i).y(), sim->getAgentVelocity(i).x());
    if(std::abs(temp_orientation-temp_velocity_tan)<0.05){
      // very close, amost no angular difference
      tb3_angular_velocity[i] = 0.0f;
    }else if(std::abs(temp_orientation-temp_velocity_tan)<PI){
      if(temp_orientation>temp_velocity_tan){ // clockwise
        tb3_angular_velocity[i] = -ANGULAR_VELOCITY;
      }else{ // couterclockwise
        tb3_angular_velocity[i] = ANGULAR_VELOCITY;
      }
    }else{
      if(temp_orientation>temp_velocity_tan){ // counter-clockwise
        tb3_angular_velocity[i] = ANGULAR_VELOCITY;
      }else{ // clockwise
        tb3_angular_velocity[i] = -ANGULAR_VELOCITY;
      }
    }
    // when angular velocity is positive, it stands for counter clockwise
    // when angular velocity is negative, it stands for clockwise
    ROS_INFO("===this is turtlebot3_%ld", i);
    ROS_INFO("temp_orientation:%f", temp_orientation);
    ROS_INFO("temp_velocity_tan:%f", temp_velocity_tan);
    ROS_INFO("linear velocity:%f", tb3_linear_velocity[i]);
    ROS_INFO("angular velocity:%f", tb3_angular_velocity[i]);
  }
}


void updateTurtlebot3Status(RVO::RVOSimulator* sim, ros::Publisher tb3_pub[]){
  geometry_msgs::Twist twist;
  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;
  for (std::size_t i = 0; i < sim->getNumAgents(); ++i){
    if(tb3_reached[i]){ // make the robot still if it reaches goal
      linear.x = 0.0f;
      angular.z = 0.0f;
    }else{
      linear.x = tb3_linear_velocity[i];
      angular.z = tb3_angular_velocity[i];
    }
    twist.linear=linear;
    twist.angular=angular;
    tb3_pub[i].publish(twist);
  }
}

void new_odom_tb3_0(const nav_msgs::Odometry::ConstPtr &msg){

  tb3_x_position[0] = msg->pose.pose.position.x;
  tb3_y_position[0] = msg->pose.pose.position.y;

  tf::Quaternion q(
  msg->pose.pose.orientation.x,
  msg->pose.pose.orientation.y,
  msg->pose.pose.orientation.z,
  msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  tb3_theta[0] = yaw;

  ROS_INFO("------tb3_0-----");
  ROS_INFO("%f", tb3_x_position[0]);
	ROS_INFO("%f", tb3_y_position[0]);
	ROS_INFO("%f", tb3_theta[0]);
  ROS_INFO("%f", msg->twist.twist.linear.x);
  ROS_INFO("%f", msg->twist.twist.angular.y);

}

void new_odom_tb3_1(const nav_msgs::Odometry::ConstPtr &msg){

  tb3_x_position[1] = msg->pose.pose.position.x;
  tb3_y_position[1] = msg->pose.pose.position.y;

  tf::Quaternion q(
  msg->pose.pose.orientation.x,
  msg->pose.pose.orientation.y,
  msg->pose.pose.orientation.z,
  msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  tb3_theta[1] = yaw;

  ROS_INFO("------tb3_1-----");
  ROS_INFO("%f", tb3_x_position[1]);
	ROS_INFO("%f", tb3_y_position[1]);
	ROS_INFO("%f", tb3_theta[1]);
  ROS_INFO("%f", msg->twist.twist.linear.x);
  ROS_INFO("%f", msg->twist.twist.angular.y);
}

bool reach_goal(RVO::RVOSimulator* sim){
  bool result = true;
  for(std::size_t i=0; i<sim->getNumAgents(); i++){
    if(tb3_reached[i]){
      continue;
    }
    if(RVO::absSq(goals[i] - sim->getAgentPosition(i)) > k_goal_parameter * sim->getAgentRadius(i) * sim->getAgentRadius(i)){
      result = false;
    }else{
      tb3_reached[i] = true;
    }
  }
  return result;
}

void test(RVO::RVOSimulator* sim){
  for(int i=0; i<sim->getNumAgents();i++){
      ROS_INFO("------TEST-----");
      ROS_INFO("%f", tb3_x_position[i]);
	    ROS_INFO("%f", tb3_y_position[i]);
      ROS_INFO("%f", sim->getAgentPosition(i).x());
      ROS_INFO("%f", sim->getAgentPosition(i).y());
      ROS_INFO("------TEST-----");
  }
}

int main(int argc, char ** argv){

	ros::init(argc, argv, "move_robot");
	ros::NodeHandle nh;
  ros::Subscriber tb3_sub[2];
  ros::Publisher tb3_pub[2];
  // notice the buffer size is one. ----------------here--------  
	tb3_sub[0] = nh.subscribe("/tb3_0/odom", 1, new_odom_tb3_0); 
  tb3_pub[0]= nh.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 1);
  tb3_sub[1] = nh.subscribe("/tb3_1/odom", 1, new_odom_tb3_1);
  tb3_pub[1] = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 1);

  RVO::RVOSimulator* sim = new RVO::RVOSimulator();
  setupScenario(sim);

  ros::Duration(1).sleep();
  // ros::spinOnce();
  // setPositionToRVO(sim);
  // test(sim);

  ros::Rate loop_rate(1/sim->getTimeStep());
  while(ros::ok())
  { 
    ros::spinOnce(); // get position of odometry of each robot
    setPositionToRVO(sim); // refresh the position in RVO by turtlebot real position
    test(sim);
    if(reach_goal(sim)){
      updateTurtlebot3Status(sim, tb3_pub); // make the speed of turtlebot to zero
      break;
    }
    setPreferredVelocities(sim); // set preferred velocity to RVO to calculate
    sim->doStep(); // calculate new velocity and new position. 
                  // !! Note new position should not be used until setPositionToRVO function
    setVelocityToTurtlebot3(sim);
    updateTurtlebot3Status(sim, tb3_pub); // make turtlebot3 move for timestep
    loop_rate.sleep();
  }

  ros::Duration(1).sleep();

	return 0;
}


