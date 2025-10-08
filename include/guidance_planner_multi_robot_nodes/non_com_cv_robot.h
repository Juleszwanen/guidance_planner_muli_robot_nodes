#pragma once

#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_types/data_types.h>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros_tools/profiling.h>
#include <geometry_msgs/Twist.h>

struct CvRobotState
{
    double _x;
    double _y;
    double _vel;
    double _psi;
    double _psi_dot;

    MPCPlanner::ReferencePath reference_path;

    void set(double x, double y, double vel, double psi, double psi_dot)
    {
        _x = x;
        _y = y;
        _vel = vel;
        _psi = psi;
        _psi_dot = psi_dot;
    }

    void set(const std::string &key, double value)
    {
        if (key == "x")
        {
            _x = value;
        }
        else if (key == "y")
        {
            _y = value;
        }
        else if (key == "v" || key == "vel")
        {
            _vel = value;
        }
        else if (key == "psi")
        {
            _psi = value;
        }
        else if (key == "psi_dot")
        {
            _psi_dot = value;
        }
        // Could add error handling here if desired
    }

    double get(const std::string &key) const
    {
        if (key == "x")
        {
            return _x;
        }
        else if (key == "y")
        {
            return _y;
        }
        else if (key == "v" || key == "vel")
        {
            return _vel;
        }
        else if (key == "psi")
        {
            return _psi;
        }
        else if (key == "psi_dot")
        {
            return _psi_dot;
        }
        // Return 0.0 for unknown keys (could add error handling here)
        return 0.0;
    }
};

class NonComCVRobot
{
private:
    double _constance_velocity{0.5};
    double _integrator_step{0.2};
    double _N{30};
    double _goal_tolerance{0.5};

    bool _objective_reached{false};

    CvRobotState _state;

    std::string _ego_robot_ns{"/jackal6"};
    int _ego_robot_id{6};
    std::string _global_frame{"map"};

    ros::NodeHandle _nh;

    // ROS Subscribers
    ros::Subscriber _state_pose_sub;
    ros::Subscriber _path_sub;

    ros::Publisher _cmd_pub;
    ros::Publisher _pose_pub;
    ros::Publisher _cv_trajectory_pub;
    ros::Publisher _reverse_roadmap_pub;

    ros::Timer _timer;
    RosTools::Timer _startup_timer;

public:
    void statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);

    void loop(const ros::TimerEvent & /*event*/);

    void initializeSubscribersAndPublishers(ros::NodeHandle &nh);
    bool isPathTheSame(const nav_msgs::Path::ConstPtr &msg) const;
    void rotatePiRadiansCw(geometry_msgs::Twist &cmd);
    bool objectiveReached();
    void purePursuitController(geometry_msgs::Twist &cmd);
    int findClosestPointOnPath();
    int findLookaheadPoint(int start_idx, double lookahead_distance);
    MPCPlanner::Prediction CreateConstantVelocityPrediction(const geometry_msgs::Twist &cmd);
    void publishTrajectory(const MPCPlanner::Prediction &prediction);
    MPCPlanner::Prediction CreateConstantVelocityPrediction();

public:
    NonComCVRobot(ros::NodeHandle &nh);
    ~NonComCVRobot();
};
