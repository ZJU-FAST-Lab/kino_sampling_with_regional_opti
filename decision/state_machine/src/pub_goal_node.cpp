#include "quadrotor_msgs/PositionCommand.h"
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "pub_goal_node");
  ros::NodeHandle nh("~");
  ros::Publisher goal_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/goal", 1);
  std::random_device rd;
  std::mt19937_64 gen_;                  
  std::uniform_real_distribution<double> pos_mean_rand_;
  gen_ = std::mt19937_64(rd());
  pos_mean_rand_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  quadrotor_msgs::PositionCommand goal;
  int pub_num = 1000; // number of samples
  ros::Rate rate(1.0/30.0);
  Eigen::Vector3d last_pt(0.0, 0.0, 0.0);
  int i(0);
  while (ros::ok() && i < pub_num)
  {
    Eigen::Vector3d pt;
    double pos_mean = pos_mean_rand_(gen_);
    pt[0] = pos_mean * 20.0; // half map size x, should be half as the arg map_size_x in simulator.launch
    pos_mean = pos_mean_rand_(gen_);
    pt[1] = pos_mean * 20.0; // half map size y, should be half as the arg map_size_y in simulator.launch
    pt[2] = 0.0;
    if ((pt - last_pt).norm() >= 5)
    {
      i++;
      std::cout << i << std::endl;
      goal.position.x = pt[0];
      goal.position.y = pt[1];
      goal.position.z = pt[2];
      goal_pub.publish(goal);
      last_pt = pt;
      rate.sleep();
    }
  }
  
}