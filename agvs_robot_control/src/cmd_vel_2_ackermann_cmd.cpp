#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <robotnik_msgs/set_mode.h>
#include <robotnik_msgs/get_mode.h>
#include <robotnik_msgs/set_odometry.h>
#include <robotnik_msgs/ptz.h>
#include "ackermann_msgs/AckermannDriveStamped.h"

#include <sstream>
#include "std_msgs/String.h"


ackermann_msgs::AckermannDriveStamped _ackermann_data;
bool is_read = false;
void subCallback(const geometry_msgs::Twist::ConstPtr& _cmd_data)
{
    is_read = true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Node_CmdVel_to_AckermannCommand");
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/agvs_robot_control_command", 50);
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, subCallback);
    ros::Rate       loop_rate(30);
    
    while (ros::ok())
    {
        if(is_read) pub.publish(_ackermann_data);   
        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
}