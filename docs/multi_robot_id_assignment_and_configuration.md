# Multi-Robot ID Assignment and Configuration Documentation

## Table of Contents
1. [Overview](#overview)
2. [ID Assignment Architecture](#id-assignment-architecture)
3. [Perspective-Based System Behavior](#perspective-based-system-behavior)
4. [Configuration Files and Parameters](#configuration-files-and-parameters)
5. [Launch File Structure](#launch-file-structure)
6. [Code Implementation Details](#code-implementation-details)
7. [Complete Configuration Flow](#complete-configuration-flow)
8. [Common Scenarios and Examples](#common-scenarios-and-examples)
9. [Troubleshooting ID Mismatches](#troubleshooting-id-mismatches)

---

## Overview

This document describes the **0-based ID assignment system** used in the multi-robot navigation framework. Understanding this system is critical for:
- Correctly configuring Vicon object tracking
- Ensuring proper obstacle filtering and trajectory association
- Debugging multi-robot coordination issues
- Adding new robots or non-communicating obstacles

### Key Principle: 0-Based Indexing Throughout

**CRITICAL**: The entire system uses **0-based indexing** to align with Vicon's object ID assignment convention:
- `jackal1` → ID **0**
- `jackal2` → ID **1**  
- `jackal3` → ID **2**
- `dynamic_object1` → ID **3** (first non-communicating object after 3 robots)
- ...and so on

---

## ID Assignment Architecture

### 1. Vicon Bundle Obstacles Convention

The **Vicon bundle_obstacles.launch** assigns IDs based on the **array position** in `dynamic_object_topics`:

```xml
<!-- From bundle_obstacles.launch -->
<arg name="dynamic_object_topics" default="[jackal1, jackal2, dynamic_object1]"/>
```

**ID Assignment Rules**:
- IDs are assigned sequentially starting from **0**
- Position in array = Object ID
- This happens in `vicon_util/obstacle_bundle_node`

**Example**:
```yaml
dynamic_object_topics: [jackal1, jackal2, dynamic_object1]
                        ↓        ↓        ↓
Object IDs:             0        1        2
```

### 2. Robot ID Extraction (extractRobotIdFromNamespace)

The function `extractRobotIdFromNamespace()` converts robot namespace to 0-based ID:

**Location**: `/workspace/src/mpc_planner/mpc_planner_util/src/multi_robot_utility_functions.cpp`

```cpp
int extractRobotIdFromNamespace(const std::string &ns)
{
    // Handle both "/jackalX" and "jackalX" formats
    // Returns 0-based index to match Vicon's 0-based object ID system
    // jackal1 -> 0, jackal2 -> 1, jackal3 -> 2, etc.
    if (ns.front() == '/')
        return std::stoi(ns.substr(7)) - 1; // skip "/jackal" and convert to 0-based
    else
        return std::stoi(ns.substr(6)) - 1; // skip "jackal" and convert to 0-based
}
```

**Conversion Examples**:
```cpp
extractRobotIdFromNamespace("/jackal1") → 0
extractRobotIdFromNamespace("/jackal2") → 1
extractRobotIdFromNamespace("jackal3")  → 2
```

### 3. Non-Communicating Object ID Generation

The function `extractIdentifierIndicesNonComObj()` generates IDs for non-communicating obstacles:

**Location**: `/workspace/src/mpc_planner/mpc_planner_util/src/multi_robot_utility_functions.cpp`

```cpp
std::vector<int> extractIdentifierIndicesNonComObj(
    const std::vector<std::string> &all_robot_namespaces, 
    const unsigned int &num_non_com_obj)
{
    std::vector<int> identifier_indices_non_com_obj;

    // Non-communicating objects start after all robot IDs
    const int start_index = all_robot_namespaces.size();
    const int end_index = start_index + num_non_com_obj;

    identifier_indices_non_com_obj.reserve(num_non_com_obj);

    // Generate sequential indices: [num_robots, num_robots + num_non_com_obj - 1]
    for (int i = start_index; i < end_index; ++i)
    {
        identifier_indices_non_com_obj.push_back(i);
    }

    return identifier_indices_non_com_obj;
}
```

**Example**:
```cpp
// Given: robot_ns_list = ["/jackal1", "/jackal2"], num_non_com_obj = 2
extractIdentifierIndicesNonComObj(robot_ns_list, 2)
  → Returns: [2, 3]
  → Meaning: dynamic_object1 = ID 2, dynamic_object2 = ID 3
```

### 4. Complete ID Assignment Table

| Entity | Array Position | extractRobotIdFromNamespace | Vicon ID | Final ID Used |
|--------|---------------|---------------------------|----------|---------------|
| jackal1 | 0 | 0 | 0 | **0** |
| jackal2 | 1 | 1 | 1 | **1** |
| dynamic_object1 | 2 | N/A | 2 | **2** |
| dynamic_object2 | 3 | N/A | 3 | **3** |

---

## Perspective-Based System Behavior

### Jackal1's Perspective

When you launch `jules_test_real_jackal_arg.launch jackal_name:=jackal1`:

#### **Ego Robot Configuration**
```yaml
Robot Name: jackal1
Namespace: /jackal1
Ego Robot ID (_ego_robot_id): 0  # From extractRobotIdFromNamespace("/jackal1")
```

#### **Other Robots in System**
```yaml
robot_ns_list: ["/jackal1", "/jackal2"]  # From multi_robot_settings.yaml

Other Robots (_other_robot_nss):
  - /jackal2  # ID: 1 (from extractRobotIdFromNamespace)

Trajectory-Based Obstacles Created:
  - _data.trajectory_dynamic_obstacles["/jackal2"]
    * ID: 1
    * Position: Far away initially
    * Updated via: trajectoryCallback when jackal2 publishes
```

#### **Vicon Objects Tracked**
```xml
<!-- From jules_test_real_jackal_arg.launch -->
<arg name="dynamic_object_topics" default="[jackal1, jackal2, dynamic_object1]"/>
```

**Object ID Assignment** (by Vicon bundle_obstacles):
```yaml
jackal1:         ID 0  # Skipped in obstacleCallback (ego robot)
jackal2:         ID 1  # Skipped in obstacleCallback (object.id < _robot_ns_list.size())
dynamic_object1: ID 2  # Processed as non-communicating obstacle
```

#### **Obstacle Processing in obstacleCallback**

**Code Location**: `jules_ros1_real_jackalplanner.cpp:338`

```cpp
for (auto &object : msg->objects)
{
    // Skip robot obstacles (they're handled by trajectoryCallback)
    if (object.id < _robot_ns_list.size())
        continue;
    
    // Process non-communicating obstacles (object.id >= 2 in this case)
    // ...
}
```

**Filtering Logic for jackal1**:
```yaml
_robot_ns_list.size(): 2
object.id < 2:
  - object.id = 0 (jackal1) → SKIP ✓
  - object.id = 1 (jackal2) → SKIP ✓
  - object.id = 2 (dynamic_object1) → PROCESS ✓
```

#### **Dynamic Obstacles Container State**

After initialization (`initializeOtherRobotsAsObstacles`):
```cpp
_data.dynamic_obstacles = [
    DynamicObstacle(index=1, position=[100,100]),  // jackal2 (trajectory-based)
    DynamicObstacle(index=2, position=[100,100])   // dynamic_object1 (non-comm)
]

_data.trajectory_dynamic_obstacles = {
    "/jackal2": DynamicObstacle(index=1, position=[100,100])
}
```

After Vicon update (obstacleCallback):
```cpp
_data.dynamic_obstacles[1].position = [2.5, 3.1]  // Updated from trajectoryCallback
_data.dynamic_obstacles[0].position = [4.2, 1.7]  // Updated from Vicon (dynamic_object1)
```

---

### Jackal2's Perspective

When you launch `jules_test_real_jackal_arg.launch jackal_name:=jackal2`:

#### **Ego Robot Configuration**
```yaml
Robot Name: jackal2
Namespace: /jackal2
Ego Robot ID (_ego_robot_id): 1  # From extractRobotIdFromNamespace("/jackal2")
```

#### **Other Robots in System**
```yaml
robot_ns_list: ["/jackal1", "/jackal2"]  # Same as jackal1

Other Robots (_other_robot_nss):
  - /jackal1  # ID: 0 (from extractRobotIdFromNamespace)

Trajectory-Based Obstacles Created:
  - _data.trajectory_dynamic_obstacles["/jackal1"]
    * ID: 0
    * Updated via: trajectoryCallback when jackal1 publishes
```

#### **Vicon Objects Tracked**
```xml
<!-- From jules_test_real_jackal_arg.launch -->
<arg name="dynamic_object_topics" default="[jackal2, jackal1, dynamic_object1]"/>
<!-- NOTE: Order changed! jackal2 first because it's the ego robot in this launch -->
```

**Object ID Assignment** (by Vicon bundle_obstacles):
```yaml
jackal2:         ID 0  # Skipped in obstacleCallback (ego robot for jackal2)
jackal1:         ID 1  # Skipped in obstacleCallback (object.id < _robot_ns_list.size())
dynamic_object1: ID 2  # Processed as non-communicating obstacle
```

**IMPORTANT**: Even though the Vicon array order changes, the **robot_ns_list** order stays the same, so:
- `_robot_ns_list.size()` is still **2**
- Filtering logic `object.id < 2` still correctly skips IDs 0 and 1

#### **Obstacle Processing in obstacleCallback**

**Filtering Logic for jackal2**:
```yaml
_robot_ns_list.size(): 2
object.id < 2:
  - object.id = 0 (jackal2 in Vicon) → SKIP ✓
  - object.id = 1 (jackal1 in Vicon) → SKIP ✓
  - object.id = 2 (dynamic_object1) → PROCESS ✓
```

---

## Configuration Files and Parameters

### 1. Multi-Robot Settings (Multi-Robot Level)

**File**: `/workspace/src/guidance_planner_multi_robot_nodes/config/multi_robot_settings.yaml`

**Purpose**: Defines the **global robot network configuration** (shared across all robots)

```yaml
# List of ALL robots in the system (order matters for ID calculation)
robot_ns_list: ["/jackal1", "/jackal2"]

# Network configuration for inter-robot communication
/jackal1:
    network:
        publisher_end_point: "tcp://192.168.0.99:3001"   
/jackal2:
    network:
        publisher_end_point: "tcp://192.168.0.99:3002"

# Dead man switch configuration
rqt_dead_man_switch: true

# Global frame for localization
frames/global: "map"
```

**Key Points**:
- `robot_ns_list` defines the **complete robot network**
- Order in `robot_ns_list` affects `_robot_ns_list.size()` used for filtering
- Loaded by: `<rosparam command="load" file="...multi_robot_settings.yaml"/>`

---

### 2. Guidance Planner Configuration (Algorithm Level)

**File**: `/workspace/src/mpc_planner/mpc_planner_jackal/config/guidance_planner.yaml`

**Purpose**: Configures the **guidance planner module** (pathfinding, topology selection)

```yaml
guidance_planner:
  T: 6.0          # Time horizon [s]
  N: 30           # Discrete time steps
  
  homotopy:
    n_paths: 4    # Number of guidance trajectories
    comparison_function: Homology
  
  max_velocity: 3.0
  max_acceleration: 3.0
  
  spline_optimization:
    enable: true
    geometric: 25.
    smoothness: 10.
    collision: 0.5
```

**Key Points**:
- Loaded by: `<rosparam command="load" file="...guidance_planner.yaml"/>`
- Affects trajectory generation, not ID assignment
- Shared across all robots (same algorithm parameters)

---

### 3. MPC Planner Settings (Control Level)

**File**: `/workspace/src/mpc_planner/mpc_planner_jackal/config/settings.yaml`

**Purpose**: Configures the **MPC solver and controller**

**Note**: This file is loaded **inside C++ code**, not via launch file:
```cpp
// From jules_ros1_real_jackalplanner.cpp constructor
Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));
```

**Typical Contents**:
```yaml
control_frequency: 10.0  # Hz
integrator_step: 0.2     # seconds
N: 30                    # MPC horizon steps

robot_radius: 0.4        # meters
obstacle_radius: 0.4     # meters
max_obstacles: 10        # maximum number of obstacles

deceleration_at_infeasible: 1.0  # m/s²

JULES:
  safe_extra_data: true
  enable_trajectory_interpolation: true
  communicate_on_topology_switch_only: false
  robot_max_velocity: 2.0
  robot_max_angular_velocity: 2.0
```

**Key Points**:
- Not directly loaded in launch file
- Loaded via `SYSTEM_CONFIG_PATH` macro during node initialization
- Contains low-level control and safety parameters

---

### 4. Launch File Parameters (Runtime Configuration)

**File**: `/workspace/src/guidance_planner_multi_robot_nodes/launch/jules_test_real_jackal_arg.launch`

**Purpose**: Robot-specific runtime parameters

```xml
<!-- Runtime parameters (set via <param> tags) -->
<param name="forward_experiment" type="bool" value="$(arg direction_of_experiment)"/>
<param name="num_non_com_obj" type="int" value="$(arg num_non_com_obj)"/>
<param name="ego_robot_ns" type="string" value="/$(arg jackal_name)" />
```

**Accessed in C++ via**:
```cpp
nh.param("/ego_robot_ns", this->_ego_robot_ns, this->_ego_robot_ns);
nh.param("/forward_experiment", this->_forward_x_experiment, this->_forward_x_experiment);
nh.param("/num_non_com_obj", this->_num_non_com_obj, this->_num_non_com_obj);
```

---

## Launch File Structure

### jules_test_real_jackal_arg.launch Architecture

```xml
<?xml version="1.0"?>
<launch>

    <!-- ============================================= -->
    <!-- SECTION 1: LAUNCH ARGUMENTS (User Inputs)    -->
    <!-- ============================================= -->
    <arg name="jackal_name" default="jackal1"/>
    <arg name="num_non_com_obj" default="0"/>
    
    <!-- Dynamically set other jackal based on current jackal -->
    <arg name="other_jackal" value="jackal2" if="$(eval arg('jackal_name') == 'jackal1')"/>
    <arg name="other_jackal" value="jackal1" if="$(eval arg('jackal_name') == 'jackal2')"/>
    
    <!-- Robot-specific roadmaps -->
    <arg name="roadmap" value="maps/mobile_robotics_lab/diagonal.xml"          
         if="$(eval arg('jackal_name') == 'jackal1')"/>
    <arg name="roadmap" value="maps/mobile_robotics_lab/diagonal_reversed.xml" 
         if="$(eval arg('jackal_name') == 'jackal2')"/>
    
    <!-- Direction of experiment -->
    <arg name="direction_of_experiment" value="true"  if="$(eval arg('jackal_name') == 'jackal1')" />
    <arg name="direction_of_experiment" value="false" if="$(eval arg('jackal_name') == 'jackal2')" />

    <!-- ============================================= -->
    <!-- SECTION 2: ROS PARAMETERS (Runtime Config)   -->
    <!-- ============================================= -->
    <param name="forward_experiment" type="bool" value="$(arg direction_of_experiment)"/>
    <param name="num_non_com_obj" type="int" value="$(arg num_non_com_obj)"/>
    <param name="ego_robot_ns" type="string" value="/$(arg jackal_name)" />

    <!-- ============================================= -->
    <!-- SECTION 3: CONFIGURATION FILES (YAML Loads)  -->
    <!-- ============================================= -->
    <!-- Multi-robot network configuration (defines robot_ns_list) -->
    <rosparam command="load" file="$(find guidance_planner_multi_robot_nodes)/config/multi_robot_settings.yaml"/>
    
    <!-- Guidance planner algorithm parameters -->
    <rosparam command="load" file="$(find mpc_planner_jackal)/config/guidance_planner.yaml"/>
    
    <!-- NOTE: MPC settings.yaml loaded inside C++ code via Configuration::getInstance() -->

    <!-- ============================================= -->
    <!-- SECTION 4: MAIN PLANNER NODE                 -->
    <!-- ============================================= -->
    <node pkg="mpc_planner_jackal" type="jules_ros1_real_jackalplanner" 
          name="jackal_planner" respawn="false" output="screen">
        
        <param name="simulation" value="false"/>
        
        <!-- Topic remappings -->
        <remap from="/input/state" to="/$(arg jackal_name)/odometry/filtered"/>
        <remap from="/input/goal" to="/roadmap/goal"/>
        <remap from="/input/reference_path" to="/roadmap/reference"/>
        <remap from="/input/obstacles" to="/vicon_util/dynamic_objects"/>
        <remap from="/input/bluetooth" to="/bluetooth_teleop/joy"/>
        <remap from="/output/command" to="/cmd_vel"/>
        
        <!-- Robot-to-robot communication topics -->
        <remap from="/robot_to_robot/output/current_trajectory" 
               to="/$(arg jackal_name)/robot_to_robot/output/current_trajectory"/>
        <remap from="/events/objective_reached" 
               to="/$(arg jackal_name)/events/objective_reached" />
        <remap from="/output/pose" 
               to="/$(arg jackal_name)/output/pose" />
    </node>

    <!-- ============================================= -->
    <!-- SECTION 5: VICON TRACKING SYSTEM             -->
    <!-- ============================================= -->
    <include file="$(find vicon_bridge)/launch/vicon.launch">
        <!-- Objects to track (ORDER DETERMINES IDs!) -->
        <arg name="object_names" value="[$(arg jackal_name), $(arg other_jackal), dynamic_object1]"/>
        
        <arg name="object_msg_types" 
             default="[geometry_msgs/PoseWithCovarianceStamped, 
                       geometry_msgs/PoseWithCovarianceStamped, 
                       geometry_msgs/PoseWithCovarianceStamped]"/>
        
        <arg name="object_frame_ids" default="[map, map, map]"/>
        
        <arg name="object_publish_topics" 
             default="[/vicon/$(arg jackal_name), 
                       /vicon/$(arg other_jackal), 
                       /vicon/dynamic_object1]"/>
        
        <arg name="object_frequency_divider" default="[2, 2, 2]"/>
    </include>

    <!-- ============================================= -->
    <!-- SECTION 6: VICON UTILITIES (EKF + BUNDLING)  -->
    <!-- ============================================= -->
    <include file="$(find vicon_util)/launch/bundle_obstacles.launch">
        <arg name="run_ekfs" value="true"/>
        <arg name="visualize_scene" value="true"/>
        <arg name="robot_topic" value="$(arg jackal_name)"/>
        
        <!-- CRITICAL: This array order determines Vicon object IDs! -->
        <arg name="dynamic_object_topics" 
             default="[$(arg jackal_name), $(arg other_jackal), dynamic_object1]"/>
        
        <arg name="dynamic_object_radius" default="0.4"/>
        <arg name="static_object_topics" default="[]"/>
        <arg name="static_object_radius" default="0.3"/>
        <arg name="static_object_sizes" default="[]"/>
    </include>

    <!-- ============================================= -->
    <!-- SECTION 7: SUPPORTING NODES                  -->
    <!-- ============================================= -->
    <!-- Robot URDF description -->
    <include file="$(find jackal_description)/launch/description.launch"/>

    <!-- Inter-robot communication middleware (ZeroMQ) -->
    <node pkg="guidance_planner_multi_robot_nodes" type="middleware_publisher_node.py" 
          name="middleware_from_ROS_to_ZEROMQ_publisher_node" respawn="false" output="screen"/>
    <node pkg="guidance_planner_multi_robot_nodes" type="middleware_subscriber_node.py" 
          name="middleware_from_ZEROMQ_to_ROS_subscriber_node" respawn="false" output="screen"/>

    <!-- Roadmap module for reference path generation -->
    <include file="$(find roadmap)/launch/roadmap.launch">
        <arg name="map_file_name" value="$(arg roadmap)"/>
    </include>

    <!-- ============================================= -->
    <!-- SECTION 8: VISUALIZATION (Conditional)       -->
    <!-- ============================================= -->
    <!-- Launch RViz only for jackal1 -->
    <node name="rviz" pkg="rviz" type="rviz" 
          args="-d $(find mpc_planner_jackal)/rviz/ros1.rviz" 
          output="screen" 
          if="$(eval arg('jackal_name') == 'jackal1')"/>

</launch>
```

---

## Code Implementation Details

### 1. Obstacle Initialization (initializeOtherRobotsAsObstacles)

**Location**: `jules_ros1_real_jackalplanner.cpp`

```cpp
bool JulesRealJackalPlanner::initializeOtherRobotsAsObstacles(
    const std::set<std::string> &other_robot_namespaces,
    MPCPlanner::RealTimeData &data,
    const double radius)
{
    // Get indices for non-communicating objects
    std::vector<int> non_com_indices = MultiRobot::extractIdentifierIndicesNonComObj(
        _robot_ns_list, _num_non_com_obj);
    
    const Eigen::Vector2d FAR_AWAY_POSITION(100.0, 100.0);
    const Eigen::Vector2d ZERO_VELOCITY(0.0, 0.0);

    // Initialize communicating robots
    for (const auto &robot_ns : other_robot_namespaces)
    {
        // Create trajectory obstacle for this robot
        data.trajectory_dynamic_obstacles.emplace(
            robot_ns,
            MPCPlanner::DynamicObstacle(
                MultiRobot::extractRobotIdFromNamespace(robot_ns),  // 0-based ID!
                FAR_AWAY_POSITION,
                0.0,
                radius));

        // Create corresponding dynamic obstacle
        MPCPlanner::DynamicObstacle dummy_obstacle = 
            data.trajectory_dynamic_obstacles.at(robot_ns);
        data.dynamic_obstacles.push_back(dummy_obstacle);

        LOG_INFO(_ego_robot_ns + ": Created obstacle for robot " + robot_ns +
                 " with index " + std::to_string(obstacle.index));
    }

    // Initialize non-communicating objects
    for (const int index : non_com_indices)
    {
        data.dynamic_obstacles.emplace_back(
            index,  // ID from extractIdentifierIndicesNonComObj
            FAR_AWAY_POSITION,
            0.0,
            CONFIG["obstacle_radius"].as<double>());

        LOG_INFO(_ego_robot_ns + ": Created non-communicating obstacle with index " +
                 std::to_string(index) + " (ID from Vicon bundle)");
    }

    return true;
}
```

**Key Points**:
- **Communicating robots**: Use `extractRobotIdFromNamespace` → 0-based IDs
- **Non-communicating objects**: Use `extractIdentifierIndicesNonComObj` → Sequential IDs after robots
- Both stored in `_data.dynamic_obstacles` vector

---

### 2. Obstacle Filtering (obstacleCallback)

**Location**: `jules_ros1_real_jackalplanner.cpp:338`

```cpp
void JulesRealJackalPlanner::obstacleCallback(
    const derived_object_msgs::ObjectArray::ConstPtr &msg)
{
    // Process non-communicating obstacles from Vicon
    for (auto &object : msg->objects)
    {
        // Skip robot obstacles (they're handled by trajectoryCallback)
        if (object.id < _robot_ns_list.size())
            continue;
        
        // Calculate velocity magnitude
        double velocity = std::sqrt(object.twist.linear.x * object.twist.linear.x +
                                    object.twist.linear.y * object.twist.linear.y);

        // Find the dynamic obstacle with matching index
        auto it = std::find_if(_data.dynamic_obstacles.begin(),
                               _data.dynamic_obstacles.end(),
                               [&object](const MPCPlanner::DynamicObstacle &obs)
                               {
                                   return obs.index == object.id;
                               });

        if (it != _data.dynamic_obstacles.end())
        {
            // Update obstacle position and prediction
            auto &dynamic_obstacle = *it;
            dynamic_obstacle.position = Eigen::Vector2d(
                object.pose.position.x, object.pose.position.y);
            
            dynamic_obstacle.prediction = MPCPlanner::getConstantVelocityPrediction(
                dynamic_obstacle.position,
                global_twist,
                CONFIG["integrator_step"].as<double>(),
                CONFIG["N"].as<int>());

            LOG_INFO(_ego_robot_ns + ": Updated non-comm obstacle ID " + 
                     std::to_string(object.id) + " at [" + 
                     std::to_string(dynamic_obstacle.position.x()) + ", " +
                     std::to_string(dynamic_obstacle.position.y()) + "]");
        }
    }
}
```

**Filtering Logic Explained**:
```cpp
if (object.id < _robot_ns_list.size())
    continue;  // Skip this object

// Example with 2 robots:
_robot_ns_list.size() = 2

object.id = 0 → Skip (jackal1, communicating robot)
object.id = 1 → Skip (jackal2, communicating robot)
object.id = 2 → Process (dynamic_object1, non-comm obstacle)
object.id = 3 → Process (dynamic_object2, non-comm obstacle)
```

---

### 3. Trajectory Association (trajectoryCallback)

**Location**: `jules_ros1_real_jackalplanner.cpp`

```cpp
void JulesRealJackalPlanner::trajectoryCallback(
    const mpc_planner_msgs::ObstacleGMM::ConstPtr &msg, 
    const std::string ns)
{
    // Get the trajectory-based obstacle for this robot namespace
    auto it = _data.trajectory_dynamic_obstacles.find(ns);
    if (it == _data.trajectory_dynamic_obstacles.end())
    {
        LOG_WARN_THROTTLE(1000, _ego_robot_ns + ": Received trajectory from " + ns +
                                    " but obstacle not initialized yet. Ignoring...");
        return;
    }

    auto &robot_trajectory_obstacle = it->second;

    // Validate ID matches
    if (robot_trajectory_obstacle.index != msg->id)
    {
        LOG_WARN(_ego_robot_ns + ": Trajectory obstacle ID mismatch for robot " + ns +
                 " - expected ID: " + std::to_string(robot_trajectory_obstacle.index) +
                 ", received ID: " + std::to_string(msg->id) + ". Ignoring...");
        return;
    }

    // Update obstacle with new trajectory prediction
    // ... (trajectory processing code)
}
```

**Key Point**: The `msg->id` field **must match** the obstacle's index, which comes from `extractRobotIdFromNamespace` (0-based).

---

## Complete Configuration Flow

### Data Flow Diagram

```
1. LAUNCH FILE ARGUMENTS
   ├── jackal_name: jackal1
   ├── num_non_com_obj: 1
   └── other_jackal: jackal2

2. YAML CONFIGURATION LOADING
   ├── multi_robot_settings.yaml → robot_ns_list: ["/jackal1", "/jackal2"]
   ├── guidance_planner.yaml → Algorithm parameters
   └── (settings.yaml loaded in C++ code)

3. VICON CONFIGURATION
   ├── object_names: [jackal1, jackal2, dynamic_object1]
   └── dynamic_object_topics: [jackal1, jackal2, dynamic_object1]
        ↓
   bundle_obstacles assigns IDs:
   ├── jackal1 → ID 0
   ├── jackal2 → ID 1
   └── dynamic_object1 → ID 2

4. C++ NODE INITIALIZATION
   ├── nh.param("/ego_robot_ns") → "/jackal1"
   ├── nh.param("/num_non_com_obj") → 1
   ├── nh.getParam("/robot_ns_list") → ["/jackal1", "/jackal2"]
   └── _ego_robot_id = extractRobotIdFromNamespace("/jackal1") → 0

5. OBSTACLE INITIALIZATION
   ├── _other_robot_nss = {"/jackal2"}
   ├── extractRobotIdFromNamespace("/jackal2") → 1
   ├── extractIdentifierIndicesNonComObj(robot_ns_list, 1) → [2]
   └── Create obstacles:
       ├── trajectory_dynamic_obstacles["/jackal2"] with ID=1
       └── dynamic_obstacles: [ID=1 (jackal2), ID=2 (dynamic_object1)]

6. RUNTIME OBSTACLE PROCESSING
   ├── obstacleCallback receives Vicon data
   ├── Filter: object.id < 2 → Skip IDs 0,1 (robots)
   ├── Process: object.id = 2 → Update dynamic_object1
   └── trajectoryCallback receives trajectory from /jackal2
       └── Validate: msg->id == 1 → Match with trajectory_dynamic_obstacles["/jackal2"]
```

---

## Common Scenarios and Examples

### Scenario 1: Two Robots, One Dynamic Object

**Launch Command**:
```bash
roslaunch guidance_planner_multi_robot_nodes jules_test_real_jackal_arg.launch \
  jackal_name:=jackal1 num_non_com_obj:=1
```

**Configuration**:
```yaml
robot_ns_list: ["/jackal1", "/jackal2"]
dynamic_object_topics: [jackal1, jackal2, dynamic_object1]
```

**ID Assignment**:
| Entity | Vicon ID | Code ID | Role |
|--------|----------|---------|------|
| jackal1 | 0 | 0 | Ego robot |
| jackal2 | 1 | 1 | Other robot (trajectory-based) |
| dynamic_object1 | 2 | 2 | Non-comm obstacle (CV prediction) |

**Jackal1's Obstacle Processing**:
```cpp
_robot_ns_list.size() = 2

obstacleCallback filtering:
  object.id=0 → Skip (jackal1, ego)
  object.id=1 → Skip (jackal2, handled by trajectoryCallback)
  object.id=2 → PROCESS (dynamic_object1)

_data.dynamic_obstacles = [
  DynamicObstacle(index=1, ...), // jackal2
  DynamicObstacle(index=2, ...)  // dynamic_object1
]
```

---

### Scenario 2: Three Robots, Two Dynamic Objects

**Launch Command**:
```bash
roslaunch guidance_planner_multi_robot_nodes jules_test_real_jackal_arg.launch \
  jackal_name:=jackal1 num_non_com_obj:=2
```

**Configuration**:
```yaml
robot_ns_list: ["/jackal1", "/jackal2", "/jackal3"]
dynamic_object_topics: [jackal1, jackal2, jackal3, dynamic_object1, dynamic_object2]
```

**ID Assignment**:
| Entity | Vicon ID | Code ID | Role |
|--------|----------|---------|------|
| jackal1 | 0 | 0 | Ego robot |
| jackal2 | 1 | 1 | Other robot (trajectory) |
| jackal3 | 2 | 2 | Other robot (trajectory) |
| dynamic_object1 | 3 | 3 | Non-comm obstacle |
| dynamic_object2 | 4 | 4 | Non-comm obstacle |

**Jackal1's Obstacle Processing**:
```cpp
_robot_ns_list.size() = 3

obstacleCallback filtering:
  object.id < 3 → Skip (robots: 0, 1, 2)
  object.id >= 3 → PROCESS (dynamic_object1=3, dynamic_object2=4)

extractIdentifierIndicesNonComObj(["/jackal1", "/jackal2", "/jackal3"], 2)
  → Returns: [3, 4]
```

---

### Scenario 3: Debugging ID Mismatch

**Problem**: Vicon object not being processed

**Diagnostic Steps**:

1. **Check Vicon bundled message**:
```bash
rostopic echo /vicon_util/dynamic_objects -n 1
```
Output:
```yaml
objects:
  - id: 0  # Should be jackal1
  - id: 1  # Should be jackal2
  - id: 2  # Should be dynamic_object1
```

2. **Check robot_ns_list**:
```bash
rosparam get /robot_ns_list
```
Output: `['/jackal1', '/jackal2']`

3. **Calculate expected filtering**:
```python
robot_ns_list_size = 2
objects_to_skip = [0, 1]  # IDs less than size
objects_to_process = [2, 3, 4, ...]  # IDs >= size
```

4. **Verify object initialization**:
```bash
# Check logs for:
"Created non-communicating obstacle with index 2"
```

5. **Check dynamic_object_topics order**:
```bash
rosparam get /obstacle_bundle_node/dynamic_objects/topics
```
Should match Vicon object_names order!

---

## Troubleshooting ID Mismatches

### Issue 1: "Could not find obstacle with ID X"

**Symptom**:
```
[WARN]: Could not find obstacle with ID 2 in dynamic_obstacles vector
```

**Diagnosis**:
```bash
# Check what IDs were initialized
grep "Created.*obstacle.*index" rosout.log

# Check Vicon object IDs
rostopic echo /vicon_util/dynamic_objects | grep "id:"
```

**Common Causes**:
1. **Mismatch between num_non_com_obj and Vicon objects**
   - Solution: Ensure `num_non_com_obj` matches actual Vicon dynamic objects

2. **Wrong dynamic_object_topics order**
   - Solution: Verify array order in bundle_obstacles.launch

3. **extractIdentifierIndicesNonComObj not called**
   - Solution: Check initializeOtherRobotsAsObstacles logs

---

### Issue 2: "Trajectory obstacle ID mismatch"

**Symptom**:
```
[WARN]: Trajectory obstacle ID mismatch for robot /jackal2 
        - expected ID: 1, received ID: 2
```

**Diagnosis**:
```bash
# Check ego robot ID calculation
echo "Ego: /jackal1"
# extractRobotIdFromNamespace("/jackal1") should return 0

# Check other robot ID
echo "Other: /jackal2"
# extractRobotIdFromNamespace("/jackal2") should return 1

# Check published trajectory ID
rostopic echo /jackal2/robot_to_robot/output/current_trajectory | grep "id:"
```

**Common Causes**:
1. **extractRobotIdFromNamespace not subtracting 1**
   - Solution: Verify implementation uses `- 1` for 0-based indexing

2. **Wrong ID in publishDirectTrajectory**
   - Solution: Check `ego_robot_trajectory_as_obstacle.id = _ego_robot_id;`

---

### Issue 3: Robots not skipped in obstacleCallback

**Symptom**:
```
[INFO]: Updated non-comm obstacle ID 1 ...
# (Should be skipped, not updated!)
```

**Diagnosis**:
```bash
# Check robot_ns_list size
rosparam get /robot_ns_list | wc -l

# Verify filtering condition
# In code: if (object.id < _robot_ns_list.size()) continue;
```

**Common Causes**:
1. **robot_ns_list not loaded properly**
   - Solution: Check `nh.getParam("/robot_ns_list", _robot_ns_list)` succeeds

2. **Filtering condition incorrect**
   - Solution: Must be `<`, not `<=` (0-based indexing!)

---

## Summary and Best Practices

### Key Takeaways

1. **0-Based Indexing Everywhere**:
   - `jackal1` → ID 0, `jackal2` → ID 1
   - Matches Vicon's array-based assignment

2. **Vicon Array Order = Object IDs**:
   - `dynamic_object_topics: [jackal1, jackal2, dynamic_object1]`
   - Results in: jackal1=0, jackal2=1, dynamic_object1=2

3. **robot_ns_list.size() is the Filtering Threshold**:
   - IDs < size → Robots (skip in obstacleCallback)
   - IDs >= size → Non-comm objects (process)

4. **Configuration Hierarchy**:
   ```
   Launch Arguments
      ↓
   ROS Parameters (runtime)
      ↓
   YAML Files (multi_robot_settings, guidance_planner)
      ↓
   C++ Configuration::getInstance() (settings.yaml)
      ↓
   Runtime Obstacle Initialization
   ```

### Configuration Checklist

When adding a new robot or obstacle:

- [ ] Update `robot_ns_list` in `multi_robot_settings.yaml`
- [ ] Update `object_names` in vicon.launch
- [ ] Update `dynamic_object_topics` in bundle_obstacles.launch
- [ ] **Ensure array orders match** (robots first, then non-comm objects)
- [ ] Set `num_non_com_obj` correctly in launch command
- [ ] Verify `extractRobotIdFromNamespace` uses `- 1` for 0-based
- [ ] Check filtering: `object.id < _robot_ns_list.size()`
- [ ] Test with `rostopic echo /vicon_util/dynamic_objects`

### Debugging Workflow

1. **Verify Vicon IDs**: `rostopic echo /vicon_util/dynamic_objects`
2. **Check robot_ns_list**: `rosparam get /robot_ns_list`
3. **Calculate threshold**: `len(robot_ns_list)` = filtering threshold
4. **Review initialization logs**: Look for "Created obstacle" messages
5. **Monitor obstacleCallback**: Check which IDs are skipped vs processed
6. **Validate trajectory IDs**: Ensure `msg->id` matches expected

---

**Document Version**: 1.0  
**Last Updated**: November 12, 2025  
**Author**: Multi-Robot Navigation Team
