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

#define ANGULAR_VELOCITY 0.3
#define PI 3.1415926


std::vector<RVO::Vector2> goals;
float tb3_x_position[2];
float tb3_y_position[2];
float tb3_theta[2];
float tb3_linear_velocity[2];
float tb3_angular_velocity[2];
bool tb3_reached[2];


void setupScenario(RVO::RVOSimulator* sim) {
  // Specify global time step of the simulation.
  sim->setTimeStep(0.25f);

  // setAgentDefaults (float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity=Vector2())
  // Specify default parameters for agents that are subsequently added.
  sim->setAgentDefaults(8.0f, 10.0f, 5.0f, 5.0f, 0.15f, 1.0f);

  // Add agents, specifying their start position.
  sim->addAgent(RVO::Vector2(-3.0f, -3.0f));
  sim->addAgent(RVO::Vector2(3.0f, -3.0f));

  // Create goals (simulator is unaware of these).
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    goals.push_back(-sim->getAgentPosition(i));
  }

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
  for(size_t i=0; i < sim->getNumAgents(); ++i){
    sim->setAgentPosition(i, RVO::Vector2(tb3_x_position[i], tb3_y_position[i]));
  }
}

void setPreferredVelocities(RVO::RVOSimulator* sim) {
  // Set the preferred velocity for each agent.
  for (std::size_t i = 0; i < sim->getNumAgents(); ++i) {
    if (RVO::absSq(goals[i] - sim->getAgentPosition(i)) < sim->getAgentRadius(i) * sim->getAgentRadius(i) ) {
      // Agent is within one radius of its goal, set preferred velocity to zero
      sim->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
    } else {
      // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
      sim->setAgentPrefVelocity(i, RVO::normalize(goals[i] - sim->getAgentPosition(i)));
    }
  }
}

void setVelocityToTurtlebot3(RVO::RVOSimulator* sim){
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    // linear velocity stands for velocity in x
    tb3_linear_velocity[i] = RVO::abs(sim->getAgentVelocity(i));

    // angular velocity is determined by orientation and velocity tang
    float temp_orientation = tb3_theta[i];
    float temp_velocity_tan = atan2(tb3_y_position[i], tb3_x_position[i]);
    if(abs(temp_orientation-temp_velocity_tan)<0.2){
      // very close, amost no angular difference
      tb3_angular_velocity[i] = 0.0f;
    }else if(abs(temp_orientation-temp_velocity_tan)<PI){
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

  ROS_INFO("%f", tb3_x_position[0]);
	ROS_INFO("%f", tb3_y_position[0]);
	ROS_INFO("%f", tb3_theta[0]);

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

  ROS_INFO("%f", tb3_x_position[1]);
	ROS_INFO("%f", tb3_y_position[1]);
	ROS_INFO("%f", tb3_theta[1]);

}

bool reach_goal(RVO::RVOSimulator* sim){
  bool result = true;
  for(std::size_t i=0; i<sim->getNumAgents(); i++){
    if(tb3_reached[i]){
      continue;
    }
    if(RVO::absSq(goals[i] - sim->getAgentPosition(i)) > sim->getAgentRadius(i) * sim->getAgentRadius(i)){
      result = false;
    }else{
      tb3_reached[i] = true;
    }
  }
  return result;
}

int main(int argc, char ** argv){

  RVO::RVOSimulator* sim = new RVO::RVOSimulator();

	ros::init(argc, argv, "move_robot");
	ros::NodeHandle nh;
  ros::Subscriber tb3_sub[2];
  ros::Publisher tb3_pub[2];
  // notice the buffer size is one. ----------------here--------  
	tb3_sub[0] = nh.subscribe("/tb3_0/odom", 1, new_odom_tb3_0); 
  tb3_pub[0]= nh.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 1);
  tb3_sub[1] = nh.subscribe("/tb3_1/odom", 1, new_odom_tb3_1);
  tb3_pub[1] = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 1);

  setupScenario(sim);

  ros::spinOnce();
  setPositionToRVO(sim);

  ros::Rate loop_rate(1/sim->getTimeStep());
  while(ros::ok() && (!reach_goal(sim)))
  { 
    setPreferredVelocities(sim); // set preferred velocity to RVO to calculate
    sim->doStep(); // calculate new velocity and new position. 
                  // !! Note new position should not be used until setPositionToRVO function
    setVelocityToTurtlebot3(sim);
    updateTurtlebot3Status(sim, tb3_pub); // make turtlebot3 move for timestep
    loop_rate.sleep();
    ros::spinOnce(); // get position of odometry of each robot
    setPositionToRVO(sim); // refresh the position in RVO by turtlebot real position
  }
  updateTurtlebot3Status(sim, tb3_pub); // make the speed of turtlebot to zero

	return 0;
}


