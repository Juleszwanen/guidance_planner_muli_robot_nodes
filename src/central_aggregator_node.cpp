

#include <guidance_planner_multi_robot_nodes/central_aggregator_node.h>
#include <mpc_planner_msgs/GetOtherTrajectories.h>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>

#include <std_msgs/Empty.h>
#include <ros_tools/profiling.h>
#include <cmath> // for

CentralAggregator::CentralAggregator(ros::NodeHandle &nh)
{

    PROFILE_FUNCTION();
    RosTools::Instrumentor::Get().BeginSession("guidance_planner_multi_robot_nodes");
    LOG_INFO("STARTING NODE: " + ros::this_node::getName());

    if (!nh.getParam("/robot_ns_list", _robot_ns_list))
    {
        LOG_ERROR("No robot_ns_list param");
    }

    else
    {
        for (const auto &ns_str : this->_robot_ns_list)
        {
            LOG_INFO("Robot namespace: " + ns_str);
        }
    }

    LOG_DIVIDER();
    // Pre-initialize cache containers for performance optimization
    for (const auto &ns : _robot_ns_list)
    {
        // Initialize empty obstacle arrays for each robot namespace
        _const_obs_cache[ns] = mpc_planner_msgs::ObstacleArray();
        _trajectory_obs_cache[ns] = mpc_planner_msgs::ObstacleArray();

        // Pre-reserve space to minimize reallocations (assume max 12 other robots)
        _const_obs_cache[ns].obstacles.reserve(12);
        _trajectory_obs_cache[ns].obstacles.reserve(12);

        // Pre-compute profiling names to avoid string concatenation in hot path
        _profiling_names[ns] = "CentralAggregator::" + ns + "_trajectoryCallback";

        _robots_objective_reached.insert({ns, false});
    }

    this->initializeSubscribersAndPublishers(nh);
}

CentralAggregator::~CentralAggregator()
{
    LOG_INFO("STOPPING NODE: " + ros::this_node::getName() + "\n");
    RosTools::Instrumentor::Get().EndSession();
}

// This function is called at the start up of the node
void CentralAggregator::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{

    // we clear the robot subcriber list for safety reasons
    this->_robot_pose_sub_list.clear();
    // We reserve vector space equal to the number of robots in our system
    this->_robot_pose_sub_list.reserve(_robot_ns_list.size());

    // Lets create a subscriber for each robot and bind a standard call back function for each one
    for (const auto &ns : _robot_ns_list)
    {
        // create the subscriber string
        // const std::string topic_pose = ns + "/output/pose";
        // LOG_INFO("Subscribing to: " + topic_pose);
        // // create the subscriber object, which wille subscribe to each robots output pose
        // auto sub_pose_i = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose, 1,
        //                                                            boost::bind(&CentralAggregator::poseCallback, this, _1, ns));
        // // add the subscriber to the list of subscribers such that the objects are not destroyed
        // this->_robot_pose_sub_list.push_back(sub_pose_i);

        // // create the subscribers for each trajectory a robot outputs
        // const std::string topic_trajectory = ns + "/output/current_trajectory";
        // LOG_INFO("Subscribing to: " + topic_trajectory);
        // auto sub_traject_i = nh.subscribe<nav_msgs::Path>(topic_trajectory, 1,
        //                                                   boost::bind(&CentralAggregator::trajectoryCallback, this, _1, ns));
        // this->_robot_trajectory_sub_list.push_back(sub_traject_i);

        const std::string topic_objective_reached = ns + "/events/objective_reached";
        LOG_INFO(ros::this_node::getName() + " subscribing to: " + topic_objective_reached);
        auto sub_objective_reached_i = nh.subscribe<std_msgs::Bool>(topic_objective_reached, 1,
                                                                    boost::bind(&CentralAggregator::objectiveReachedCallback, this, _1, ns));

        this->_robot_objective_reached_sub_list.push_back(sub_objective_reached_i);
    }

    // Create all the publisher objects and store them in _obs_pub_by_ns <- of type map
    _obs_pub_by_ns.clear();
    _obs_trajectory_pub_by_ns.clear();

    // Jules: uncomment if you want to use the server style communication
    // for (const auto &ns : _robot_ns_list)
    // {
    //     // Create the publishers which are for the constant velocity obstacles
    //     const std::string output_topic = ns + "/input/obstacles";
    //     LOG_INFO("Advertising: " + output_topic);
    //     _obs_pub_by_ns[ns] = nh.advertise<mpc_planner_msgs::ObstacleArray>(output_topic, 1);

    //     // create the publishers which are responsible for trajectory obstacles
    //     const std::string output_trajectory_topic = ns + "/input/trajectory_obstacles";
    //     LOG_INFO("Advertising: " + output_trajectory_topic);
    //     _obs_trajectory_pub_by_ns[ns] = nh.advertise<mpc_planner_msgs::ObstacleArray>(output_trajectory_topic, 1);
    // }

    const std::string all_robots_reached_objective_topic = "all_robots_reached_objective";
    LOG_INFO("Advertising: " + all_robots_reached_objective_topic);
    _objectives_reached_pub = nh.advertise<std_msgs::Bool>(all_robots_reached_objective_topic, 1);

    // Optionally read params (keep your defaults if not set)
    nh.param("publish_rate_hz", _publish_rate_hz, _publish_rate_hz);
    nh.param("stale_timeout_s", _stale_timeout_s, _stale_timeout_s);
    nh.param("global_frame", _global_frame, _global_frame);
    nh.param("robot_prediction_horizon", _robot_prediction_horizon, _robot_prediction_horizon);
    nh.param("robot_prediction_step", _robot_prediction_step, _robot_prediction_step);

    //
    // _trajectory_service = nh.advertiseService("get_other_robot_obstacles_srv", &CentralAggregator::trajectoryServiceFunction, this);

    // Start periodic publishing, but why would I periodicly publish this node and not just publish it whenever I want
    // _timer = nh.createTimer(
    //     ros::Duration(1.0 / std::max(1.0, 20.0)),
    //     &CentralAggregator::timerCallback,
    //     this);
}

// this callback fills the map/dictionary with prediction objects per robot
void CentralAggregator::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg,
                                     const std::string ns)
{
    // Get or create the per-robot prediction slot
    auto &robot_prediction = _robots_predictions[ns];

    // Give it a stable id once (based on namespace); keep it for the node lifetime
    if (robot_prediction.id < 0)
    {
        robot_prediction.id = getJackalNumber(ns); // or your own hash if needed
    }

    // Reset trajectory contents but keep the id
    robot_prediction.clearRobotPredictionButKeepId();
    // Use message timestamp if available, otherwise current time
    robot_prediction.received_msg_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    // Decode state from your encoded PoseStamped
    const double psi = msg->pose.orientation.z; // encoded yaw
    const double v = msg->pose.position.z;      // encoded speed
    const double vx = std::cos(psi) * v;
    const double vy = std::sin(psi) * v;

    Eigen::Vector2d pos(msg->pose.position.x, msg->pose.position.y);

    // Current state (k = 0)
    robot_prediction.Add(
        pos,
        psi,
        Eigen::Vector2d(vx, vy) // uncertainty left at defaults (0)
    );

    ROS_DEBUG_STREAM_THROTTLE(1.0,
                              "Pose from " << ns
                                           << " x=" << pos.x()
                                           << " y=" << pos.y()
                                           << " v=" << v
                                           << " psi=" << psi
                                           << " t=" << msg->header.stamp.toSec());

    // Predict N steps ahead with constant velocity (optional)
    if (_robot_prediction_horizon > 0)
    {
        double px = pos.x();
        double py = pos.y();

        // Reserve space to avoid reallocations during prediction horizon loop
        robot_prediction.reserve(robot_prediction.pos.size() + _robot_prediction_horizon);

        for (int k = 0; k < _robot_prediction_horizon; ++k)
        {
            px += vx * _robot_prediction_step; // new x
            py += vy * _robot_prediction_step; // new y

            robot_prediction.Add(
                Eigen::Vector2d(px, py),
                psi,                    // keep heading constant
                Eigen::Vector2d(vx, vy) // keep velocity constant
                // major/minor axes left at defaults (0 → deterministic)
            );
        }
    }
}

// This function is called whenever there is a message published on the ns + "/output/current_trajectory"; topic
void CentralAggregator::trajectoryCallback(const nav_msgs::Path::ConstPtr &msg,
                                           const std::string ns)
{
    // Use pre-computed profiling name to avoid string concatenation in hot path
    const std::string &profiling_name = _profiling_names[ns];
    LOG_DEBUG(profiling_name);
    PROFILE_SCOPE(profiling_name.c_str());

    // get an alreadu existing RobotPrediction object or create a new instance of RobotPrediction object if the namespace does not yet exist
    auto &robot_trajectory = _robots_trajectory_predictions[ns];
    if (robot_trajectory.id < 0)
    {
        robot_trajectory.id = this->getJackalNumber(ns);
    }

    // Reset trajectory contents but keep the id
    robot_trajectory.clearRobotPredictionButKeepId(); //

    // Use message timestamp if available, otherwise current time
    robot_trajectory.received_msg_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    // Get the list of posses stored in the message
    const auto &list_of_posesStamped = msg->poses;
    if (_robot_prediction_horizon > 0 && !list_of_posesStamped.empty())
    {
        // Reserve space to avoid reallocations during trajectory parsing
        // Since we just cleared the vectors, size is 0, so just reserve for incoming trajectory
        const size_t trajectory_size = list_of_posesStamped.size();
        robot_trajectory.reserve(trajectory_size);

        for (const auto &poseStamped : list_of_posesStamped) // Use const reference to avoid copying
        {
            robot_trajectory.Add(
                Eigen::Vector2d(poseStamped.pose.position.x, poseStamped.pose.position.y),
                RosTools::quaternionToAngle(poseStamped.pose),
                Eigen::Vector2d(-1, -1) // we dont use the velocity here so we set it to -1, can later be adjusted
                // major/minor axes left at defaults (0 → deterministic)
            );
        }
    }
}

void CentralAggregator::timerCallback(const ros::TimerEvent &)
{
    LOG_DEBUG("----- timerCallback Central Aggregator -----");
    PROFILE_SCOPE("CentralAggregator::timerCallback");

    // Single timestamp for the entire update cycle to reduce system calls
    const ros::Time current_time = ros::Time::now();

    // Clear and reuse pre-allocated caches instead of creating new containers
    for (auto &[ns, array] : _const_obs_cache)
    {
        array.obstacles.clear();
        // Keep reserved capacity, just clear contents
    }
    for (auto &[ns, array] : _trajectory_obs_cache)
    {
        array.obstacles.clear();
        // Keep reserved capacity, just clear contents
    }

    // Populate the pre-allocated caches using single timestamp
    populateConstantObstacleArrays(current_time);
    populateTrajectoryObstacleArrays(current_time);

    // Publish from the pre-allocated caches
    for (const auto &ns : _robot_ns_list)
    {
        auto it_constant_obstacle_pub = _obs_pub_by_ns.find(ns);
        auto it_trajectory_obstacle_pub = _obs_trajectory_pub_by_ns.find(ns);

        if (it_constant_obstacle_pub == _obs_pub_by_ns.end() || it_trajectory_obstacle_pub == _obs_trajectory_pub_by_ns.end())
        {
            continue;
        }

        // Publish directly from caches (no map lookups needed)
        it_constant_obstacle_pub->second.publish(_const_obs_cache[ns]);
        it_trajectory_obstacle_pub->second.publish(_trajectory_obs_cache[ns]);
    }

    LOG_DEBUG("----- end timerCallback Central Aggregator -----");
}

void CentralAggregator::objectiveReachedCallback(const std_msgs::Bool::ConstPtr &msg, const std::string ns)
{
    bool reached_objective = msg->data;
    if (!reached_objective)
    {
        LOG_ERROR(ns + " sends to CentralAggregator that it has reached its objective but the message contains false, something is wrong.. data: " + std::to_string(reached_objective));
        return;
    }

    _robots_objective_reached[ns] = true;
    if (allRobotsReachedObjective())
    {
        std_msgs::Bool event;
        event.data = true;
        _objectives_reached_pub.publish(event);
        LOG_INFO("All Robots have reached their objective, time to reverse paths and start again.........");
        LOG_DIVIDER();
        resetRobotsObjectiveReached();
    }
}

int CentralAggregator::getJackalNumber(const std::string &ns)
{
    // Handle both "/jackalX" and "jackalX" formats
    if (ns.front() == '/')
        return std::stoi(ns.substr(7)); // skip "/jackal"
    else
        return std::stoi(ns.substr(6)); // skip "jackal"
}

bool CentralAggregator::allRobotsReachedObjective()
{
    bool all_robots_reached_objective = true;
    for (const auto [ns, reach_objective_bool] : _robots_objective_reached)
    {
        if (!reach_objective_bool)
        {
            LOG_DEBUG(ns + " has not reached its objective yet");
            all_robots_reached_objective = false;
            return all_robots_reached_objective;
        }
    }

    return all_robots_reached_objective;
}

void CentralAggregator::resetRobotsObjectiveReached()
{
    for (auto &[ns, reach_objective_bool] : _robots_objective_reached)
    {
        reach_objective_bool = false;
    }
}
// Performance optimized function that populates pre-allocated constant obstacle arrays
void CentralAggregator::populateConstantObstacleArrays(const ros::Time &timestamp)
{
    for (const auto &robot_ns : _robot_ns_list)
    {
        auto &obstacle_array_msg = _const_obs_cache[robot_ns]; // Direct reference to pre-allocated cache

        // obstacles already cleared in timerCallback, header needs updating
        obstacle_array_msg.header.stamp = timestamp;
        obstacle_array_msg.header.frame_id = _global_frame;

        for (const auto &kv : _robots_predictions)
        {
            const std::string &other_ns = kv.first;
            const RobotPrediction &prediction = kv.second;

            // Early filtering for performance
            if (other_ns == robot_ns)
                continue;
            if (prediction.pos.empty() || prediction.angle.empty())
                continue;

            // Build obstacle directly in the pre-allocated vector
            obstacle_array_msg.obstacles.emplace_back();
            auto &obstacle = obstacle_array_msg.obstacles.back();

            obstacle.id = prediction.id;

            // Current pose (k = 0)
            obstacle.pose.position.x = prediction.pos[0](0);
            obstacle.pose.position.y = prediction.pos[0](1);
            obstacle.pose.position.z = 0.0;
            obstacle.pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);

            // Future predictions
            obstacle.gaussians.emplace_back();
            auto &gaussian = obstacle.gaussians.back();

            if (prediction.pos.size() > 1)
            {
                // Reserve space for trajectory points to minimize reallocations
                gaussian.mean.poses.reserve(prediction.pos.size() - 1);
                gaussian.major_semiaxis.reserve(prediction.pos.size() - 1);
                gaussian.minor_semiaxis.reserve(prediction.pos.size() - 1);

                for (size_t i = 1; i < prediction.pos.size(); ++i)
                {
                    gaussian.mean.poses.emplace_back();
                    auto &pose = gaussian.mean.poses.back();

                    pose.pose.position.x = prediction.pos[i](0);
                    pose.pose.position.y = prediction.pos[i](1);
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation = RosTools::angleToQuaternion(prediction.angle[i]);

                    gaussian.major_semiaxis.push_back(
                        i < prediction.major_axis.size() ? prediction.major_axis[i] : 0.0);
                    gaussian.minor_semiaxis.push_back(
                        i < prediction.minor_axis.size() ? prediction.minor_axis[i] : 0.0);
                }
            }

            obstacle.probabilities.push_back(1.0);
        }
    }
}

// Performance optimized function that populates pre-allocated trajectory obstacle arrays
void CentralAggregator::populateTrajectoryObstacleArrays(const ros::Time &timestamp)
{
    for (const auto &robot_ns : _robot_ns_list)
    {
        auto &obstacle_array_msg = _trajectory_obs_cache[robot_ns]; // Direct reference to pre-allocated cache

        // obstacles already cleared in timerCallback, header needs updating
        obstacle_array_msg.header.stamp = timestamp;
        obstacle_array_msg.header.frame_id = _global_frame;

        for (const auto &kv : _robots_trajectory_predictions)
        {
            const std::string &other_ns = kv.first;
            const RobotPrediction &prediction = kv.second;

            // Early filtering for performance
            if (other_ns == robot_ns)
                continue;
            if (prediction.pos.empty() || prediction.angle.empty())
                continue;

            // Build obstacle directly in the pre-allocated vector
            obstacle_array_msg.obstacles.emplace_back();
            auto &obstacle = obstacle_array_msg.obstacles.back();

            obstacle.id = prediction.id;

            // Current pose (k = 0)
            obstacle.pose.position.x = prediction.pos[0](0);
            obstacle.pose.position.y = prediction.pos[0](1);
            obstacle.pose.position.z = 1.0; // Different Z for trajectory obstacles
            obstacle.pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);

            // Future predictions
            obstacle.gaussians.emplace_back();
            auto &gaussian = obstacle.gaussians.back();

            if (prediction.pos.size() > 1)
            {
                // Reserve space for all trajectory points to minimize reallocations
                gaussian.mean.poses.reserve(prediction.pos.size());
                gaussian.major_semiaxis.reserve(prediction.pos.size());
                gaussian.minor_semiaxis.reserve(prediction.pos.size());

                // Include ALL trajectory points (including current position at i=0)
                for (size_t i = 0; i < prediction.pos.size(); ++i)
                {
                    gaussian.mean.poses.emplace_back();
                    auto &pose = gaussian.mean.poses.back();

                    pose.pose.position.x = prediction.pos[i](0);
                    pose.pose.position.y = prediction.pos[i](1);
                    pose.pose.position.z = 1.0;
                    pose.pose.orientation = RosTools::angleToQuaternion(prediction.angle[i]);

                    gaussian.major_semiaxis.push_back(
                        i < prediction.major_axis.size() ? prediction.major_axis[i] : 0.0);
                    gaussian.minor_semiaxis.push_back(
                        i < prediction.minor_axis.size() ? prediction.minor_axis[i] : 0.0);
                }
            }

            obstacle.probabilities.push_back(1.0);
        }
    }
}

void CentralAggregator::updateNsTrajectoryObstacleArray(const ros::Time &timestamp, const std::string &ego_robot_ns)
{

    auto &obstacle_array_msg = _trajectory_obs_cache[ego_robot_ns]; // Direct reference to pre-allocated cache

    // obstacles already cleared in timerCallback, header needs updating
    obstacle_array_msg.header.stamp = timestamp;
    obstacle_array_msg.header.frame_id = _global_frame;

    for (const auto &kv : _robots_trajectory_predictions)
    {
        const std::string &other_ns = kv.first;
        const RobotPrediction &prediction = kv.second;

        // Early filtering for performance
        if (other_ns == ego_robot_ns)
            continue;
        if (prediction.pos.empty() || prediction.angle.empty())
            continue;

        // Build obstacle directly in the pre-allocated vector
        obstacle_array_msg.obstacles.emplace_back();
        auto &obstacle = obstacle_array_msg.obstacles.back();

        obstacle.id = prediction.id;

        // Current pose (k = 0)
        obstacle.pose.position.x = prediction.pos[0](0);
        obstacle.pose.position.y = prediction.pos[0](1);
        obstacle.pose.position.z = 1.0; // Different Z for trajectory obstacles
        obstacle.pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);

        // Future predictions
        obstacle.gaussians.emplace_back();
        auto &gaussian = obstacle.gaussians.back();

        if (prediction.pos.size() > 1)
        {
            // Reserve space for all trajectory points to minimize reallocations
            gaussian.mean.poses.reserve(prediction.pos.size());
            gaussian.major_semiaxis.reserve(prediction.pos.size());
            gaussian.minor_semiaxis.reserve(prediction.pos.size());

            // Include ALL trajectory points (including current position at i=0)
            for (size_t i = 0; i < prediction.pos.size(); ++i)
            {
                gaussian.mean.poses.emplace_back();
                auto &pose = gaussian.mean.poses.back();

                pose.pose.position.x = prediction.pos[i](0);
                pose.pose.position.y = prediction.pos[i](1);
                pose.pose.position.z = 1.0;
                pose.pose.orientation = RosTools::angleToQuaternion(prediction.angle[i]);

                gaussian.major_semiaxis.push_back(
                    i < prediction.major_axis.size() ? prediction.major_axis[i] : 0.0);
                gaussian.minor_semiaxis.push_back(
                    i < prediction.minor_axis.size() ? prediction.minor_axis[i] : 0.0);
            }
        }

        obstacle.probabilities.push_back(1.0);
    }
}

bool CentralAggregator::trajectoryServiceFunction(mpc_planner_msgs::GetOtherTrajectories::Request &req, mpc_planner_msgs::GetOtherTrajectories::Response &res)
{
    const std::string &ns_ego_robot = req.requesting_robot_id;
    // _robots_trajectory_predictions[ns];

    if (_trajectory_obs_cache.find(ns_ego_robot) == _trajectory_obs_cache.end())
    {
        LOG_ERROR("Unknown robot namespace: " + ns_ego_robot);
        return false;
    }

    auto &obstacle_array_msg = _trajectory_obs_cache[ns_ego_robot];

    obstacle_array_msg.obstacles.clear(); // clear the elements of this obstacle array but keep the capacity

    ros::Time current_time = ros::Time::now();
    this->updateNsTrajectoryObstacleArray(current_time, ns_ego_robot); // update the specific chache of the obstacleTrajectories

    res.obstacle_trajectories = _trajectory_obs_cache[ns_ego_robot];

    return true;
}

// This function can be used together with the timer which converts the predictions into obstacle arrays
bool CentralAggregator::trajectoryServiceTimerFunction(mpc_planner_msgs::GetOtherTrajectories::Request &req, mpc_planner_msgs::GetOtherTrajectories::Response &res)
{
    const std::string &ns_ego_robot = req.requesting_robot_id;

    // Validate robot namespace
    if (_trajectory_obs_cache.find(ns_ego_robot) == _trajectory_obs_cache.end())
    {
        LOG_ERROR("Unknown robot namespace: " + ns_ego_robot);
        return false;
    }

    // Return pre-computed data from timer
    res.obstacle_trajectories = _trajectory_obs_cache[ns_ego_robot];
    return true;
}

int main(int argc, char *argv[])
{

    // the node name will be the name which you give in the launch file
    ros::init(argc, argv, "central_aggregator");

    ros::NodeHandle nh;
    auto central_aggregator = std::make_shared<CentralAggregator>(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}