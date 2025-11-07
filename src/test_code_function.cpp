#include "mpc_planner_types/data_types.h"
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include <ros_tools/logging.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "send_cmd_vel_to_jackal");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd;
    cmd.angular.z = 0.5;
    pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    while (ros::ok())
    {
        pub_cmd.publish(cmd);
    }

    return 0;
}
