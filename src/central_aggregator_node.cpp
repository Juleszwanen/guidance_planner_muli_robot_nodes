

#include <guidance_planner_multi_robot_nodes/central_aggregator_node.h>
#include <mpc_planner_msgs/GetOtherTrajectories.h>
#include <mpc_planner_types/multi_robot_utility_functions.h>

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
    // Initialize objective tracking for communicating robots only
    for (const auto &ns : _robot_ns_list)
    {
        /** @note /jackal6 is the non communicating robot - exclude from objective tracking */
        if (ns == "/jackal6" || ns == "jackal6")
        {
            continue;
        }
        else
        {
            _robots_objective_reached.insert({ns, false});
        }
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
    // Optionally read params (keep your defaults if not set)
    nh.param("publish_rate_hz", _publish_rate_hz, _publish_rate_hz);
    nh.param("stale_timeout_s", _stale_timeout_s, _stale_timeout_s);
    nh.param("global_frame", _global_frame, _global_frame);
    nh.param("robot_prediction_horizon", _robot_prediction_horizon, _robot_prediction_horizon);
    nh.param("robot_prediction_step", _robot_prediction_step, _robot_prediction_step);
    nh.param("baseline/mode", _baseline_mode, _baseline_mode);

    // we clear the robot subcriber list for safety reasons
    this->_robot_pose_sub_list.clear();
    // We reserve vector space equal to the number of robots in our system
    this->_robot_pose_sub_list.reserve(_robot_ns_list.size());

    // Lets create a subscriber for each robot and bind a standard call back function for each one
    for (const auto &ns : _robot_ns_list)
    {
        // Subscribe to robot state for all robots (needed for CV predictions)
        const std::string topic_pose = ns + "/robot_state";
        LOG_INFO("Subscribing to: " + topic_pose);
        auto sub_pose_i = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose, 1,
                                                                   boost::bind(&CentralAggregator::poseCallback, this, _1, ns));
        this->_robot_pose_sub_list.push_back(sub_pose_i);

        // Only subscribe to objective_reached for tracked robots (exclude jackal6)
        if (ns != "/jackal6" && ns != "jackal6")
        {
            const std::string topic_objective_reached = ns + "/events/objective_reached";
            LOG_INFO(ros::this_node::getName() + " subscribing to: " + topic_objective_reached);
            auto sub_objective_reached_i = nh.subscribe<std_msgs::Bool>(topic_objective_reached, 1,
                                                                        boost::bind(&CentralAggregator::objectiveReachedCallback, this, _1, ns));
            this->_robot_objective_reached_sub_list.push_back(sub_objective_reached_i);
        }
    }
    
    const std::string all_robots_reached_objective_topic = "all_robots_reached_objective";
    LOG_INFO("Advertising: " + all_robots_reached_objective_topic);
    _objectives_reached_pub = nh.advertise<std_msgs::Bool>(all_robots_reached_objective_topic, 1);

    // Constant velocity baseline obstacle publisher
    if(_baseline_mode == "constant_velocity")
    {
        const std::string cv_obstacles_topic = "/constant_velocity_obstacles";
        LOG_INFO("Advertising: " + cv_obstacles_topic);
        _cv_obstacles_pub = nh.advertise<mpc_planner_msgs::ObstacleArray>(cv_obstacles_topic, 10);

    // Start constant velocity obstacle publishing at 40 Hz (faster than trajectory updates)
    
        _cv_timer = nh.createTimer(
            ros::Duration(1.0 / 40.0),  // 40 Hz for smooth CV baseline
            &CentralAggregator::cvTimerCallback,
            this);
        
        LOG_INFO("Constant velocity obstacle publishing initialized at 40 Hz");
    }
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
        robot_prediction.id = MultiRobot::extractRobotIdFromNamespace(ns);
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

    // Predict future states: k=1 to k=N-1 (current state k=0 already added)
    // Total points = N (matching MPC horizon)
    if (_robot_prediction_horizon > 1)
    {
        double px = pos.x();
        double py = pos.y();

        // Reserve space for N-1 future predictions (k=1 to k=N-1)
        robot_prediction.reserve(_robot_prediction_horizon);

        for (int k = 1; k < _robot_prediction_horizon; ++k)
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

void CentralAggregator::objectiveReachedCallback(const std_msgs::Bool::ConstPtr &msg, const std::string ns)
{
    // Ignore jackal6 (non-communicating robot) - should not happen due to subscription filter
    if (ns == "/jackal6" || ns == "jackal6")
    {
        LOG_WARN("Received objective_reached from non-tracked robot: " + ns + " - ignoring");
        return;
    }
    
    bool reached_objective = msg->data;
    if (!reached_objective)
    {
        LOG_ERROR(ns + " sends to CentralAggregator that it has reached its objective but the message contains false, something is wrong.. data: " + std::to_string(reached_objective));
        return;
    }

    LOG_WARN("CA NODE received reached objective message from " + ns);
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

// Constant velocity timer callback - publishes CV obstacles at fixed rate (40 Hz)
void CentralAggregator::cvTimerCallback(const ros::TimerEvent &)
{
    LOG_DEBUG("----- cvTimerCallback Central Aggregator -----");
    PROFILE_SCOPE("CentralAggregator::cvTimerCallback");
    
    publishConstantVelocityObstacles();
    
    LOG_DEBUG("----- end cvTimerCallback Central Aggregator -----");
}

// Publishes constant velocity obstacle predictions for baseline experiments
void CentralAggregator::publishConstantVelocityObstacles()
{
    // Clear previous obstacles but keep capacity
    _cv_obstacles_msg.obstacles.clear();
    
    // Set message header
    _cv_obstacles_msg.header.stamp = ros::Time::now();
    _cv_obstacles_msg.header.frame_id = _global_frame;
    
    // For each robot, create an obstacle with CV prediction
    for (const auto &kv : _robots_predictions)
    {
        // const std::string &robot_ns = kv.first;
        const RobotPrediction &prediction = kv.second;
        
        // Skip if no prediction data available
        if (prediction.pos.empty() || prediction.angle.empty())
            continue;
        
        // Create obstacle message
        mpc_planner_msgs::ObstacleGMM obstacle;
        obstacle.id = prediction.id;
        
        // Current pose (k = 0)
        obstacle.pose.position.x = prediction.pos[0](0);
        obstacle.pose.position.y = prediction.pos[0](1);
        obstacle.pose.position.z = 0.0;  // Ground level
        obstacle.pose.orientation = RosTools::angleToQuaternion(prediction.angle[0]);
        
        // Add CV prediction if horizon > 1
        if (prediction.pos.size() > 1)
        {
            obstacle.gaussians.emplace_back();
            auto &gaussian = obstacle.gaussians.back();
            
            // Reserve space for all N predictions (k=0 to k=N-1)
            gaussian.mean.poses.reserve(prediction.pos.size());
            gaussian.major_semiaxis.reserve(prediction.pos.size());
            gaussian.minor_semiaxis.reserve(prediction.pos.size());
            
            // Add all predictions: k=0 to k=N-1 (including current state)
            for (size_t k = 0; k < prediction.pos.size(); ++k)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = prediction.pos[k](0);
                pose.pose.position.y = prediction.pos[k](1);
                pose.pose.position.z = k * _robot_prediction_step;  // Encode time: k=1→0.2s, k=2→0.4s, ..., k=29→5.8s
                pose.pose.orientation = RosTools::angleToQuaternion(prediction.angle[k]);
                
                gaussian.mean.poses.push_back(pose);
                gaussian.major_semiaxis.push_back(0.0);  // Deterministic (no uncertainty)
                gaussian.minor_semiaxis.push_back(0.0);
            }
            
            obstacle.probabilities.push_back(1.0);  // Single mode, probability 1.0
        }
        
        _cv_obstacles_msg.obstacles.push_back(obstacle);
    }
    
    // Publish to all subscribers
    _cv_obstacles_pub.publish(_cv_obstacles_msg);
    
    LOG_DEBUG("Published " + std::to_string(_cv_obstacles_msg.obstacles.size()) + " constant velocity obstacles");
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