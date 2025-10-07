#include <guidance_planner_multi_robot_nodes/non_com_cv_robot.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <mpc_planner_msgs/ObstacleGMM.h>
#include <std_msgs/Empty.h>
#include <ros_tools/visuals.h>
#include <cmath>
#include <algorithm>
#include <boost/bind.hpp>

NonComCVRobot::NonComCVRobot(ros::NodeHandle &nh) : _nh(nh)
{
    LOG_INFO(_ego_robot_ns + " Starting the a non communicating constant velocity robot");
    _ego_robot_ns = ros::this_node::getNamespace();
    // Initialize subscribers and publishers
    initializeSubscribersAndPublishers(nh);

    _startup_timer.setDuration(2.0);
    _startup_timer.start();

    // Start control timer
    _timer = nh.createTimer(ros::Duration(0.1), &NonComCVRobot::loop, this); // 10 Hz
}

NonComCVRobot::~NonComCVRobot()
{
    LOG_INFO(_ego_robot_ns + " Starting the a non communicating constant velocity robot");
}

void NonComCVRobot::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Write planar pose directly from the encoded message fields.
    _state.set("x", msg->pose.position.x);
    _state.set("y", msg->pose.position.y);
    _state.set("psi", msg->pose.orientation.z); // encoded yaw (NOT a quaternion)

    // Linear speed v is encoded in position.z by upstream publisher.
    _state.set("v", msg->pose.position.z);
}

void NonComCVRobot::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    LOG_DEBUG("Path callback");

    if (isPathTheSame(msg))
        return;

    _state.reference_path.clear();

    for (const auto &pose : msg->poses)
    {
        _state.reference_path.x.push_back(pose.pose.position.x);
        _state.reference_path.y.push_back(pose.pose.position.y);
        _state.reference_path.psi.push_back(_state.get("psi")); /** @note Jules dit heb jij zelf toegevoegd*/
    }
}

bool NonComCVRobot::isPathTheSame(const nav_msgs::Path::ConstPtr &msg) const
{
    // Quick reject: if lengths differ, treat as different.
    if (_state.reference_path.x.size() != msg->poses.size())
        return false;

    // Compare only the first few points for speed/stability.
    const int num_points = std::min(2, static_cast<int>(_state.reference_path.x.size()));
    for (int i = 0; i < num_points; ++i)
    {
        // `pointInPath(i, x, y)` should apply the planner’s internal tolerance.
        if (!_state.reference_path.pointInPath(
                i,
                msg->poses[i].pose.position.x,
                msg->poses[i].pose.position.y))
        {
            return false; // diverges early → treat as a new path
        }
    }

    // Same length and first points match within tolerance → consider unchanged.
    return true;
}

void NonComCVRobot::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{
    _state_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("input/state_pose", 1,
                                                               boost::bind(&NonComCVRobot::statePoseCallback, this, _1));

    _path_sub = nh.subscribe<nav_msgs::Path>("input/reference_path", 1,
                                             boost::bind(&NonComCVRobot::pathCallback, this, _1));

    _cmd_pub = nh.advertise<geometry_msgs::Twist>("output/command", 1);
    _cv_trajectory_pub = nh.advertise<mpc_planner_msgs::ObstacleGMM>("robot_to_robot/output/current_trajectory", 1);
    _reverse_roadmap_pub = nh.advertise<std_msgs::Empty>("roadmap/reverse", 1);
}

void NonComCVRobot::loop(const ros::TimerEvent & /*event*/)
{
    if (!_startup_timer.hasFinished())
    {
        LOG_INFO_THROTTLE(1000, _ego_robot_ns + ": In startup period, skipping planning");
        return;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    // cmd.linear.y = 0.0;
    // cmd.linear.z = 0.0;
    // cmd.angular.x = 0.0;
    // cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    // Check if we have a valid reference path
    if (_state.reference_path.empty())
    {
        LOG_WARN_THROTTLE(1000, _ego_robot_ns + ": No reference path available");
        _cmd_pub.publish(cmd);
        return;
    }

    if (objectiveReached())
    {
        if (!_objective_reached)
        {
            // First time reaching objective
            LOG_INFO(_ego_robot_ns + ": Objective reached! Starting rotation and path reversal");
            _objective_reached = true;

            // Signal roadmap reversal
            std_msgs::Empty empty_msg;
            _reverse_roadmap_pub.publish(empty_msg);
        }

        // Rotate pi radians clockwise
        rotatePiRadiansCw(cmd);
    }
    else
    {
        // Reset objective reached flag if we're not at objective
        _objective_reached = false;

        // Follow path using pure pursuit controller
        purePursuitController(cmd);
    }

    // Publish command
    _cmd_pub.publish(cmd);
}

bool NonComCVRobot::objectiveReached()
{
    // Check if reference path is empty
    if (_state.reference_path.empty())
    {
        LOG_WARN(_ego_robot_ns + "Trying to work with an empty reference path");
        return false;
    }

    // Check if we reached the last point in the reference path within 0.5 m tolerance
    double dx = _state.get("x") - _state.reference_path.x.back();
    double dy = _state.get("y") - _state.reference_path.y.back();
    double distance = std::sqrt(dx * dx + dy * dy);

    return (distance < _goal_tolerance);
}

void NonComCVRobot::rotatePiRadiansCw(geometry_msgs::Twist &cmd)
{
    LOG_INFO_THROTTLE(500, _ego_robot_ns + ": Rotating pi radians clockwise");

    // Safety check
    if (_state.reference_path.psi.empty())
    {
        LOG_WARN(_ego_robot_ns + ": No reference path available for rotation");
        return;
    }

    // Get current heading from reference path (last available point)
    const double current_ref_heading = _state.reference_path.psi.back();
    const double goal_angle = current_ref_heading + M_PI; // Rotate 180 degrees
    double angle_diff = goal_angle - _state.get("psi");

    // Normalize angle difference
    while (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI)
        angle_diff += 2 * M_PI;

    LOG_VALUE_DEBUG(_ego_robot_ns + " psi:", _state.get("psi"));
    LOG_VALUE_DEBUG(_ego_robot_ns + " goal_angle:", goal_angle);
    LOG_VALUE_DEBUG(_ego_robot_ns + " angle_diff:", angle_diff);

    if (std::abs(angle_diff) > 0.1) // 0.1 rad ≈ 5.7 degrees tolerance
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 1.1 * (angle_diff > 0 ? 1.0 : -1.0); // Simple sign function
    }
    else
    {
        // Rotation complete, stop
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        LOG_INFO_THROTTLE(2000, _ego_robot_ns + ": Rotation complete. Waiting for reversed path");
    }
}

void NonComCVRobot::purePursuitController(geometry_msgs::Twist &cmd)
{
    // Find closest point on path
    int closest_idx = findClosestPointOnPath();
    if (closest_idx == -1)
    {
        LOG_WARN_THROTTLE(1000, _ego_robot_ns + ": Could not find closest point on path");
        return;
    }

    // Calculate lookahead point
    double lookahead_distance = 1.0; // 1 meter lookahead
    int target_idx = findLookaheadPoint(closest_idx, lookahead_distance);

    if (target_idx == -1)
    {
        // Use the last point if we can't find a lookahead point
        target_idx = _state.reference_path.x.size() - 1;
    }

    // Get target point
    double target_x = _state.reference_path.x[target_idx];
    double target_y = _state.reference_path.y[target_idx];

    // Calculate steering angle using pure pursuit
    double dx = target_x - _state.get("x");
    double dy = target_y - _state.get("y");
    double target_angle = std::atan2(dy, dx);

    double heading_error = target_angle - _state.get("psi");

    // Normalize angle
    while (heading_error > M_PI)
        heading_error -= 2 * M_PI;
    while (heading_error < -M_PI)
        heading_error += 2 * M_PI;

    // Apply control
    cmd.linear.x = _constance_velocity;  // Constant forward velocity
    cmd.angular.z = 1.2 * heading_error; // Proportional steering control

    // Limit angular velocity
    const double max_angular_vel = 1.5; // rad/s
    if (cmd.angular.z > max_angular_vel)
        cmd.angular.z = max_angular_vel;
    if (cmd.angular.z < -max_angular_vel)
        cmd.angular.z = -max_angular_vel;

    LOG_VALUE_DEBUG(_ego_robot_ns + " heading_error:", heading_error);
    LOG_VALUE_DEBUG(_ego_robot_ns + " target_idx:", target_idx);
}

int NonComCVRobot::findClosestPointOnPath()
{
    if (_state.reference_path.empty())
        return -1;

    double min_distance = std::numeric_limits<double>::max();
    int closest_idx = -1;

    double robot_x = _state.get("x");
    double robot_y = _state.get("y");

    for (size_t i = 0; i < _state.reference_path.x.size(); ++i)
    {
        double dx = _state.reference_path.x[i] - robot_x;
        double dy = _state.reference_path.y[i] - robot_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_idx = static_cast<int>(i);
        }
    }

    return closest_idx;
}

int NonComCVRobot::findLookaheadPoint(int start_idx, double lookahead_distance)
{
    if (_state.reference_path.empty() || start_idx < 0)
        return -1;

    double robot_x = _state.get("x");
    double robot_y = _state.get("y");

    // Start from the closest point and move forward along the path
    for (size_t i = start_idx; i < _state.reference_path.x.size(); ++i)
    {
        double dx = _state.reference_path.x[i] - robot_x;
        double dy = _state.reference_path.y[i] - robot_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance >= lookahead_distance)
        {
            return static_cast<int>(i);
        }
    }

    // If no point found at lookahead distance, return the last point
    return static_cast<int>(_state.reference_path.x.size() - 1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "num_com_cv_robot");

    ros::NodeHandle nh;
    VISUALS.init(&nh);
    auto num_com_cv_robot = std::make_shared<NonComCVRobot>(nh);
    ros::spin();

    return 0;
}