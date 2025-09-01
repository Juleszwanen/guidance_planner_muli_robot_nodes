#include <guidance_planner_multi_robot_nodes/central_aggregator_node.h>

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
    ROS_INFO_STREAM("STARTING NODE: " << ros::this_node::getName());

   
    if (!nh.getParam("/robot_ns_list", _robot_ns_list))
    {
        ROS_ERROR("No robot_ns_list param");
    }

    else
    {
        for (const auto &ns_str : this->_robot_ns_list)
        {
            ROS_INFO_STREAM("Robot namespace: " << ns_str);
        }
    }

    this->initializeSubscribersAndPublishers(nh);
}

CentralAggregator::~CentralAggregator()
{
    ROS_INFO_STREAM("STOPPING NODE: " << ros::this_node::getName() + "\n");
    
}

// This function is called at the start up of the node
void CentralAggregator::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{
    // we clear the robot subcriber list for safety reasons
    this->_robot_pose_sub_list.clear();
    // We reserve vector space equal to the number of robots in our system
    this->_robot_pose_sub_list.reserve(_robot_ns_list.size());

    // Lest create a subscriber for each robot and bind a standard call back function for each one
    for (const auto &ns : _robot_ns_list)
    {
        // create the subscriber string
        const std::string topic_pose = ns + "/output/pose";
        ROS_INFO_STREAM("Subscribing to: " << topic_pose);
        // create the subscriber object, which wille subscribe to each robots output pose
        auto sub_pose_i = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose, 5,
                                                                   boost::bind(&CentralAggregator::poseCallback, this, _1, ns));
        // add the subscriber to the list of subscribers such that the objects are not destroyed
        this->_robot_pose_sub_list.push_back(sub_pose_i);

        const std::string topic_trajectory = ns + "/output/current_trajectory";
        ROS_INFO_STREAM("Subscribing to: " << topic_trajectory);
        auto sub_traject_i = nh.subscribe<nav_msgs::Path>(topic_trajectory, 5,
                                                          boost::bind(&CentralAggregator::trajectoryCallback, this, _1, ns));
        this->_robot_trajectory_sub_list.push_back(sub_traject_i);
    }

    // Create all the publisher objects and store them in _obs_pub_by_ns <- of type map
    _obs_pub_by_ns.clear();
    _obs_trajectory_pub_by_ns.clear();
    for (const auto &ns : _robot_ns_list)
    {
        // Create the publishers which are for the constant velocity obstacles
        const std::string output_topic = ns + "/input/obstacles";
        ROS_INFO_STREAM("Advertising: " << output_topic);
        _obs_pub_by_ns[ns] = nh.advertise<mpc_planner_msgs::ObstacleArray>(output_topic, 1);

        // create the publishers which are verantwoordelijjk voor trajectory obstacles
        const std::string output_trajectory_topic = ns + "/input/trajectory_obstacles";
        ROS_INFO_STREAM("Advertising: " << output_trajectory_topic);
        _obs_trajectory_pub_by_ns[ns] = nh.advertise<mpc_planner_msgs::ObstacleArray>(output_trajectory_topic, 1);
    }

    // Optionally read params (keep your defaults if not set)
    nh.param("publish_rate_hz", _publish_rate_hz, _publish_rate_hz);
    nh.param("stale_timeout_s", _stale_timeout_s, _stale_timeout_s);
    nh.param("global_frame", _global_frame, _global_frame);
    nh.param("robot_prediction_horizon", _robot_prediction_horizon, _robot_prediction_horizon);
    nh.param("robot_prediction_step", _robot_prediction_step, _robot_prediction_step);

    // Start periodic publishing, but why would I periodicly publish this node and not just publish it whenever I want
    _timer = nh.createTimer(
        ros::Duration(1.0 / std::max(1.0, _publish_rate_hz)),
        &CentralAggregator::timerCallback,
        this);
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
    robot_prediction.clearRobotPredictionButKeepId(); // <-- if your method is named clearRobotPrediction(), use that name

    robot_prediction.received_msg_time = ros::Time::now();

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

        // (Optional) reserve to avoid reallocs
        // robot_prediction.pos.reserve(robot_prediction.pos.size() + _robot_prediction_horizon);
        // robot_prediction.angle.reserve(robot_prediction.angle.size() + _robot_prediction_horizon);
        // robot_prediction.vel.reserve(robot_prediction.vel.size() + _robot_prediction_horizon);

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

    // get or create a new instance of RobotPrediction object
    auto &robot_trajectory = _robots_trajectory_predictions[ns];
    if (robot_trajectory.id < 0)
    {
        robot_trajectory.id = this->getJackalNumber(ns);
    }

    // Reset trajectory contents but keep the id
    robot_trajectory.clearRobotPredictionButKeepId(); //

    robot_trajectory.received_msg_time = ros::Time::now();
    const auto &list_of_posesStamped = msg->poses;
    if (_robot_prediction_horizon > 0)
    {
        for (auto poseStamped : list_of_posesStamped)
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
    
    auto const_obs_array_per_robot = this->robotsToObstacleArray();

    auto trajectory_obs_array_per_robot = this->trajectoriesToObstacleArray();

    for (const auto &ns : _robot_ns_list)
    {
        auto it_constant_obstacle_pub = _obs_pub_by_ns.find(ns);              // look for the correct publisher which corresponds to the namespace we are currently searching for
        auto it_trajectory_obstacle_pub = _obs_trajectory_pub_by_ns.find(ns); // look for the correct trajectory publisher

        if (it_constant_obstacle_pub == _obs_pub_by_ns.end() || it_trajectory_obstacle_pub == _obs_trajectory_pub_by_ns.end()) // If we do not find either, we skip
        {
            continue;
        }

        auto it_constant_obstacle_msg = const_obs_array_per_robot.find(ns);        // look for the correct obstacle array msg which corresponds to the namespace we are searching for
        auto it_trajectory_obstacle_msg = trajectory_obs_array_per_robot.find(ns); // look for the correct trajectory obstacle array msg

        if (it_constant_obstacle_msg == const_obs_array_per_robot.end() || it_trajectory_obstacle_msg == trajectory_obs_array_per_robot.end())
        {
            continue;
        }

        // Publish constant velocity obstacles to the regular obstacles topic
        it_constant_obstacle_pub->second.publish(it_constant_obstacle_msg->second);

        // Publish trajectory-based obstacles to the trajectory obstacles topic
        it_trajectory_obstacle_pub->second.publish(it_trajectory_obstacle_msg->second);
    }
    
}

std::map<std::string, mpc_planner_msgs::ObstacleArray>
CentralAggregator::robotsToObstacleArray()
{
    std::map<std::string, mpc_planner_msgs::ObstacleArray> obstacle_array_msg_map;
    const ros::Time now = ros::Time::now();

    for (const auto &robot_ns : _robot_ns_list)
    {
        mpc_planner_msgs::ObstacleArray obstacle_array_msg;

        for (const auto &kv : _robots_predictions)
        {
            const std::string &other_ns = kv.first;
            const RobotPrediction &prediction = kv.second;

            // Skip ourselves
            if (other_ns == robot_ns)
                continue;

            // Drop stale sources
            // if ((now - prediction.received_msg_time).toSec() > _stale_timeout_s)
            //     continue;

            // Need at least current pose
            if (prediction.pos.empty() || prediction.angle.empty())
                continue;

            // ---- Build one obstacle (same style as pedestrian sim) ----
            obstacle_array_msg.obstacles.emplace_back();
            obstacle_array_msg.obstacles.back().id = prediction.id;

            // Current pose (k = 0)
            obstacle_array_msg.obstacles.back().pose.position.x = prediction.pos[0](0);
            obstacle_array_msg.obstacles.back().pose.position.y = prediction.pos[0](1);
            obstacle_array_msg.obstacles.back().pose.position.z = 0.0;
            obstacle_array_msg.obstacles.back().pose.orientation =
                RosTools::angleToQuaternion(prediction.angle[0]);

            // One Gaussian mode with future means (k = 1..N)
            obstacle_array_msg.obstacles.back().gaussians.emplace_back();
            auto &gaussian = obstacle_array_msg.obstacles.back().gaussians.back();

            if (prediction.pos.size() > 1)
            {
                for (size_t i = 1; i < prediction.pos.size(); ++i)
                {
                    gaussian.mean.poses.emplace_back();
                    gaussian.mean.poses.back().pose.position.x = prediction.pos[i](0);
                    gaussian.mean.poses.back().pose.position.y = prediction.pos[i](1);
                    gaussian.mean.poses.back().pose.position.z = 0.0;
                    gaussian.mean.poses.back().pose.orientation =
                        RosTools::angleToQuaternion(prediction.angle[i]);

                    // Uncertainty (keep deterministic if not provided)
                    // Voeg de onzekerheidsellips toe voor deze voorspelde positie
                    // Dit vertelt de MPC hoe zeker we zijn over deze voorspelling
                    gaussian.major_semiaxis.push_back(
                        i < prediction.major_axis.size() ? prediction.major_axis[i] : 0.0);
                    gaussian.minor_semiaxis.push_back(
                        i < prediction.minor_axis.size() ? prediction.minor_axis[i] : 0.0);
                }
            }

            // Single-mode probability
            obstacle_array_msg.obstacles.back().probabilities.push_back(1.0);
        }

        obstacle_array_msg.header.stamp = now;
        obstacle_array_msg.header.frame_id = _global_frame;

        obstacle_array_msg_map[robot_ns] = std::move(obstacle_array_msg);
    }

    return obstacle_array_msg_map;
}

std::map<std::string, mpc_planner_msgs::ObstacleArray>
CentralAggregator::trajectoriesToObstacleArray()
{

    std::map<std::string, mpc_planner_msgs::ObstacleArray> obstacle_array_msg_map;
    const ros::Time now = ros::Time::now();

    for (const auto &robot_ns : _robot_ns_list)
    {
        mpc_planner_msgs::ObstacleArray obstacle_array_msg; // for each other robot we create an empty obstacle array
        for (const auto &kv : _robots_trajectory_predictions)
        {
            const std::string &other_ns = kv.first;
            const RobotPrediction &prediction = kv.second;

            if (other_ns == robot_ns)
            {
                continue;
            }

            if (prediction.pos.empty() || prediction.angle.empty())
            {
                continue;
            }

            // ---- Build one obstacle (same style as pedestrian sim) ----
            obstacle_array_msg.obstacles.emplace_back();
            obstacle_array_msg.obstacles.back().id = prediction.id;
            // Current pose (k = 0)
            obstacle_array_msg.obstacles.back().pose.position.x = prediction.pos[0](0);
            obstacle_array_msg.obstacles.back().pose.position.y = prediction.pos[0](1);
            obstacle_array_msg.obstacles.back().pose.position.z = 0.0;
            obstacle_array_msg.obstacles.back().pose.orientation =
                RosTools::angleToQuaternion(prediction.angle[0]);

            // One Gaussian mode with future means (k = 1..N)
            obstacle_array_msg.obstacles.back().gaussians.emplace_back();
            auto &gaussian = obstacle_array_msg.obstacles.back().gaussians.back();

            if (prediction.pos.size() > 1)
            {
                for (size_t i = 1; i < prediction.pos.size(); ++i)
                {
                    gaussian.mean.poses.emplace_back();
                    gaussian.mean.poses.back().pose.position.x = prediction.pos[i](0);
                    gaussian.mean.poses.back().pose.position.y = prediction.pos[i](1);
                    gaussian.mean.poses.back().pose.position.z = 0.0;
                    gaussian.mean.poses.back().pose.orientation =
                        RosTools::angleToQuaternion(prediction.angle[i]);

                    // Uncertainty (keep deterministic if not provided)
                    // Voeg de onzekerheidsellips toe voor deze voorspelde positie
                    // Dit vertelt de MPC hoe zeker we zijn over deze voorspelling
                    gaussian.major_semiaxis.push_back(
                        i < prediction.major_axis.size() ? prediction.major_axis[i] : 0.0);
                    gaussian.minor_semiaxis.push_back(
                        i < prediction.minor_axis.size() ? prediction.minor_axis[i] : 0.0);
                }
            }

            // Single-mode probability
            obstacle_array_msg.obstacles.back().probabilities.push_back(1.0);
        }
        obstacle_array_msg.header.stamp = now;
        obstacle_array_msg.header.frame_id = _global_frame;

        obstacle_array_msg_map[robot_ns] = std::move(obstacle_array_msg);
    }

    return obstacle_array_msg_map;
}

int CentralAggregator::getJackalNumber(const std::string &ns)
{
    // Handle both "/jackalX" and "jackalX" formats
    if (ns.front() == '/')
        return std::stoi(ns.substr(7)); // skip "/jackal"
    else
        return std::stoi(ns.substr(6)); // skip "jackal"
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
