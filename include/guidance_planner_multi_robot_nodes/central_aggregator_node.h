#pragma once

#include <mpc_planner_msgs/ObstacleArray.h>
#include <mpc_planner_msgs/GetOtherTrajectories.h>
#include <ros_tools/profiling.h>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <derived_object_msgs/ObjectArray.h>
// #include <sensor_msgs/Joy.h>

// #include <std_srvs/Empty.h>
// #include <robot_localization/SetPose.h>

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>
#include <map>

// struct RobotSate
// {
//     geometry_msgs::PoseStamped pose;
//     // perhaps asl store the velocity here
//     ros::Time received_msg_time;
//     bool have{false};
//     // Here could also be something like MPCPlanner::Trajectory trajectory
// };

// Will contain the either complete trajectories of predicitions on robot trajectories
struct RobotPrediction
{
    int id{-1};
    ros::Time received_msg_time;
    std::vector<Eigen::Vector2d> pos;
    std::vector<double> angle;
    std::vector<Eigen::Vector2d> vel;
    std::vector<double> major_axis;
    std::vector<double> minor_axis;
    bool                updated_robot_prediction{false};

    RobotPrediction() = default;

    void Add(const Eigen::Vector2d &p, const double a, const Eigen::Vector2d &v,
             const double _major_axis = 0, const double _minor_axis = 0)
    {
        pos.push_back(p);
        angle.push_back(a);
        vel.push_back(v);
        major_axis.push_back(_major_axis);
        minor_axis.push_back(_minor_axis);
    }

    // Reserve space in all vectors to minimize reallocations
    void reserve(size_t capacity)
    {
        pos.reserve(capacity);
        angle.reserve(capacity);
        vel.reserve(capacity);
        major_axis.reserve(capacity);
        minor_axis.reserve(capacity);
    }

    // clear everything except the id, because that is not neccessary
    void clearRobotPrediction()
    {
        id = -1;
        pos.clear();
        angle.clear();
        vel.clear();
        major_axis.clear();
        minor_axis.clear();
        received_msg_time = ros::Time(0); // optional: reset to "zero" time
    }

    void clearRobotPredictionButKeepId()
    {
        pos.clear();
        angle.clear();
        vel.clear();
        major_axis.clear();
        minor_axis.clear();
        received_msg_time = ros::Time(0); // optional: reset to "zero" time
    }
};

class CentralAggregator
{
private:
    // Params - make sure they are visible on the parameter server, otherwise it will be really hard detect them
    double _publish_rate_hz{20.0};
    double _stale_timeout_s{0.5};
    std::string _global_frame{"map"};
    int _robot_prediction_horizon{60};
    double _robot_prediction_step{0.2};
    bool _send_info_first_time{false};

    // config /runtime
    std::vector<std::string> _robot_ns_list; // List  of robot namespaces, this will be set via a parameter.

    std::vector<ros::Subscriber> _robot_pose_sub_list;       // List of robot pose subcribers
    std::vector<ros::Subscriber> _robot_trajectory_sub_list; // List of robot trajectory subscribers
    std::map<std::string, ros::Publisher> _obs_pub_by_ns;    // A dictionary which stores the publisher by namespace,so you can easily retrieve them
    std::map<std::string, ros::Publisher> _obs_trajectory_pub_by_ns;

    ros::ServiceServer _trajectory_service;
    ros::Timer _timer; // publish loop

    std::map<std::string, RobotPrediction> _robots_predictions;            // will contain for each robot its prediction over a time horizon. This is filled inside the callback for each robot each time the callback is called
    std::map<std::string, RobotPrediction> _robots_trajectory_predictions; // will contain for each robot the trajectories. This is filled inside the callback for each robot each time the callback is called

    // Performance optimization: Pre-allocated containers to avoid repeated memory allocation in hot path
    std::map<std::string, mpc_planner_msgs::ObstacleArray> _const_obs_cache;      // Reused for robotsToObstacleArray()
    std::map<std::string, mpc_planner_msgs::ObstacleArray> _trajectory_obs_cache; // Reused for trajectoriesToObstacleArray()

    // Pre-computed strings to avoid string concatenation in hot path callbacks
    std::map<std::string, std::string> _profiling_names; // Cache for "CentralAggregator::{ns}_trajectoryCallback"

public:
    explicit CentralAggregator(ros::NodeHandle &nh);
    ~CentralAggregator();

    void initializeSubscribersAndPublishers(ros::NodeHandle &nh);

    // One callback reused for all robots; we bind `ns` per-subscriber
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, const std::string ns);
    void trajectoryCallback(const nav_msgs::Path::ConstPtr &msg, const std::string ns);
    void timerCallback(const ros::TimerEvent &);

    // Performance optimized versions that populate pre-allocated caches
    void populateConstantObstacleArrays(const ros::Time &timestamp);
    void populateTrajectoryObstacleArrays(const ros::Time &timestamp);

    // Updates the obstacle array message only for a specific namespace, this is used by the service server
    void updateNsTrajectoryObstacleArray(const ros::Time &timestamp, const std::string &robot_ns);
    bool trajectoryServiceFunction(mpc_planner_msgs::GetOtherTrajectories::Request &req, mpc_planner_msgs::GetOtherTrajectories::Response &response);
    bool trajectoryServiceTimerFunction(mpc_planner_msgs::GetOtherTrajectories::Request &req, mpc_planner_msgs::GetOtherTrajectories::Response &res);

    // get the number behind the jackal namespace so /jackal1 returns 1
    int getJackalNumber(const std::string &ns);
};
