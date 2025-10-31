# Vicon Motion Capture System Documentation

## Table of Contents
1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Vicon Bridge Package](#vicon-bridge-package)
4. [Vicon Util Package](#vicon-util-package)
5. [Integration with ros1_jackal.launch](#integration-with-ros1_jackallaunch)
6. [Data Flow](#data-flow)
7. [Configuration Guide](#configuration-guide)
8. [Troubleshooting](#troubleshooting)
9. [Performance Considerations](#performance-considerations)

## Overview

The Vicon motion capture system integration in the ROS1 Jackal project provides precise 6-DOF pose tracking for multiple robots and dynamic objects in real-time. The system consists of two main packages:

- **vicon_bridge**: Interfaces with the Vicon DataStream API to receive raw tracking data
- **vicon_util**: Processes and enhances Vicon data with velocity estimation and obstacle bundling

This documentation explains how these packages work together in the context of the `ros1_jackal.launch` file to provide localization and obstacle detection for autonomous navigation.

## System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Vicon Nexus/   │    │  vicon_bridge   │    │   vicon_util    │
│  Tracker System │───▶│     Node        │───▶│    Nodes        │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │                        │
                              ▼                        ▼
                       ┌─────────────────┐    ┌─────────────────┐
                       │ Raw Pose Topics │    │  EKF Filtered   │
                       │ /vicon/object   │    │   Odometry      │
                       │                 │    │ /object/odom/   │
                       └─────────────────┘    │   filtered      │
                                              └─────────────────┘
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │ Bundled Objects │
                                              │ /vicon_util/    │
                                              │dynamic_objects  │
                                              └─────────────────┘
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │ MPC Planner     │
                                              │ (Obstacle       │
                                              │  Avoidance)     │
                                              └─────────────────┘
```

## Vicon Bridge Package

### Purpose
The vicon_bridge package serves as the primary interface between the Vicon motion capture system and ROS. It establishes a connection to the Vicon DataStream server and publishes pose information for tracked objects.

### Key Components

#### 1. Main Node: `vicon_bridge`
- **Executable**: `vicon_bridge`
- **Configuration File**: `/workspace/src/vicon_bridge/launch/vicon.launch`
- **Source Code**: `/workspace/src/vicon_bridge/src/vicon_bridge.cpp`

#### 2. Connection Parameters
```xml
<!-- IP and port on Vicon Windows PC -->
<arg name="datastream_hostport" default="192.168.0.232"/>
```

#### 3. Object Configuration
The bridge can track multiple objects simultaneously with individual configurations:

```xml
<arg name="object_names" value="[jackal3, dynamic_object1, dynamic_object2, ...]"/>
<arg name="object_msg_types" default="[geometry_msgs/PoseWithCovarianceStamped, ...]"/>
<arg name="object_frame_ids" default="[map, map, map, ...]"/>
<arg name="object_publish_topics" default="[/vicon/jackal3, /vicon/dynamic_object1, ...]"/>
<arg name="object_frequency_divider" default="[2, 2, 2, ...]"/>
```

### Published Topics

#### Object Pose Topics
- **Topic Pattern**: `/vicon/{object_name}`
- **Message Type**: `geometry_msgs/PoseWithCovarianceStamped`
- **Frequency**: Configurable via `object_frequency_divider`
- **Frame**: Specified by `object_frame_ids` (typically `map`)

#### Transform Publications
- **TF Frames**: `vicon/{object_name}/{object_name}`
- **Reference Frame**: Configurable via `frame_id` parameter

### Configuration in ros1_jackal.launch

In the `ros1_jackal.launch` file, the vicon_bridge is configured to track:

```xml
<include file="$(find vicon_bridge)/launch/vicon.launch">
    <arg name="object_names" value="[$(arg jackal_name), dynamic_object1, dynamic_object2, dynamic_object3, dynamic_object4, dynamic_object5, dingo1]"/>
    <arg name="object_msg_types" default="[geometry_msgs/PoseWithCovarianceStamped, geometry_msgs/PoseWithCovarianceStamped, geometry_msgs/PoseWithCovarianceStamped, geometry_msgs/PoseWithCovarianceStamped, geometry_msgs/PoseWithCovarianceStamped, geometry_msgs/PoseWithCovarianceStamped, geometry_msgs/PoseWithCovarianceStamped]"/>
    <arg name="object_frame_ids" default="[map, map, map, map, map, map, map]"/>
    <arg name="object_publish_topics" default="[/vicon/$(arg jackal_name), /vicon/dynamic_object1, /vicon/dynamic_object2, /vicon/dynamic_object3, /vicon/dynamic_object4, /vicon/dynamic_object5, /vicon/dingo1]"/>
    <arg name="object_frequency_divider" default="[2, 2, 2, 2, 2, 2, 2]"/>
</include>
```

This configuration:
- Tracks the ego robot (jackal3 by default)
- Tracks 5 dynamic objects (potential obstacles/other robots)
- Tracks 1 additional robot (dingo1)
- Publishes all data at half the Vicon system frequency (divider = 2)

## Vicon Util Package

### Purpose
The vicon_util package enhances raw Vicon pose data by:
1. Adding velocity estimation through Extended Kalman Filters (EKF)
2. Bundling multiple objects into a single obstacle message
3. Providing visualization capabilities

### Key Components

#### 1. EKF Launch System
- **Script**: `/workspace/src/vicon_util/scripts/launch_ekfs.py`
- **Launch File**: `/workspace/src/vicon_util/launch/launch_ekfs.launch`
- **Configuration**: `/workspace/src/vicon_util/config/ekf.yaml`

#### 2. Obstacle Bundle Node
- **Executable**: `obstacle_bundle_node`
- **Source**: `/workspace/src/vicon_util/src/obstacle_bundle_node.cpp`
- **Launch File**: `/workspace/src/vicon_util/launch/bundle_obstacles.launch`

#### 3. Visualization Node
- **Executable**: `visualize_scene`
- **Source**: `/workspace/src/vicon_util/src/visualize_scene.cpp`

### EKF System Details

#### Launch Process
The EKF system uses a Python script to dynamically launch multiple EKF nodes:

```python
def launch_ekf(input, output, publish_tf):
    source_path = f"{os.path.dirname(os.path.realpath(__file__))}/../../../devel/setup.bash"
    run_command = f"source {source_path}"
    run_command += f" && roslaunch vicon_util ekf.launch input_topic:={input} output_topic:={output} publish_tf:={publish_tf} &"
    subprocess.call(["bash", "-c", run_command])

def node_function():
    topic_list = rospy.get_param("~input_topics", ["drone1", "dynamic_object1"])
    robot_topic = rospy.get_param("~robot_topic", "drone1")
    for topic in topic_list:
        launch_ekf(topic, topic, topic == robot_topic)
```

#### EKF Configuration
Each EKF node is configured with:

```yaml
frequency: 50
sensor_timeout: 0.5
two_d_mode: true

pose0: "pose"
pose0_config: [true, true, true,      # x, y, z position
              true, true, true,       # roll, pitch, yaw
              false, false, false,    # x, y, z velocity (estimated)
              false, false, false,    # roll, pitch, yaw velocity
              false, false, false]    # x, y, z acceleration

use_control: false  # Disabled to avoid missing configuration errors

world_frame: map
map_frame: map
base_link_frame: base_link
```

#### Topic Remapping
Each EKF node creates the following topic mappings:
- **Input**: `/vicon/{object_name}` → `pose`
- **Output**: `/{object_name}/odometry/filtered`

#### Transform Publishing
- **Robot objects**: Publish TF transforms (`publish_tf=true`)
- **Non-robot objects**: Use custom base_link frame (`publish_tf=false`)

### Obstacle Bundle System

#### Purpose
The obstacle bundle node aggregates multiple object odometry topics into a single `derived_object_msgs/ObjectArray` message for efficient obstacle processing.

#### Input Processing
The node handles two types of input messages:

1. **Odometry Messages** (`nav_msgs/Odometry`):
```cpp
void odometryToObject(nav_msgs::Odometry::ConstPtr msg, derived_object_msgs::Object &object)
{
    object.pose = msg->pose.pose;
    object.twist = msg->twist.twist;
    object.header = msg->header;
}
```

2. **Pose with Covariance** (`geometry_msgs/PoseWithCovarianceStamped`):
```cpp
void poseWithCovarianceStampedToObject(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg, derived_object_msgs::Object &object)
{
    object.pose = msg->pose.pose;
    object.twist.linear.x = 0.;  // No velocity information available
    object.twist.linear.y = 0.;
    object.twist.linear.z = 0.;
    object.header = msg->header;
}
```

#### Object Classification
Objects are classified into two categories:

1. **Dynamic Objects**: 
   - Shape: Cylinder
   - Radius: Configurable (`dynamic_object_radius`)
   - Input: EKF-filtered odometry with velocity estimates

2. **Static Objects**:
   - Shape: Box
   - Size: Configurable (`static_object_sizes`)
   - Radius: Configurable (`static_object_radius`)

#### Published Topic
- **Topic**: `/vicon_util/dynamic_objects`
- **Message Type**: `derived_object_msgs/ObjectArray`
- **Content**: All tracked objects with pose, velocity, and shape information

### Configuration in ros1_jackal.launch

```xml
<include file="$(find vicon_util)/launch/bundle_obstacles.launch">
    <arg name="run_ekfs" value="true"/>
    <arg name="visualize_scene" value="true"/>
    <arg name="robot_topic" value="$(arg jackal_name)"/>
    <arg name="dynamic_object_topics" default="[$(arg jackal_name), dynamic_object1, dynamic_object2, dynamic_object3, dynamic_object4, dynamic_object5]"/>
    <arg name="dynamic_object_radius" default="0.4"/>
    <arg name="static_object_topics" default="[]"/>
    <arg name="static_object_radius" default="0.3"/>
    <arg name="static_object_sizes" default="[]"/>
</include>
```

This configuration:
- Enables EKF filtering for all dynamic objects
- Enables scene visualization
- Treats jackal3 as the ego robot (publishes TF)
- Treats 5 objects as dynamic obstacles (cylinders with 0.4m radius)
- No static obstacles configured

## Integration with ros1_jackal.launch

### Complete Data Flow

1. **Vicon System** → **vicon_bridge**:
   - Vicon cameras track objects in 3D space
   - Raw pose data streamed via DataStream API

2. **vicon_bridge** → **vicon_util/EKF**:
   - Publishes `/vicon/{object_name}` topics
   - Data includes pose with covariance

3. **EKF Nodes** → **Obstacle Bundle**:
   - Each object gets velocity estimation
   - Publishes `/{object_name}/odometry/filtered`

4. **Obstacle Bundle** → **MPC Planner**:
   - Aggregates all objects into single message
   - Publishes `/vicon_util/dynamic_objects`

5. **MPC Planner** receives obstacles:
```xml
<remap from="/input/obstacles" to="vicon_util/dynamic_objects"/>
```

### Robot State Integration

The ego robot (jackal3) receives its state from the EKF:
```xml
<remap from="/input/state" to="/$(arg jackal_name)/odometry/filtered"/>
```

This provides:
- **Position**: From Vicon pose measurement
- **Velocity**: Estimated by EKF
- **Covariance**: Uncertainty estimates

### Transform Tree

The system creates a comprehensive transform tree:

```
map
├── base_link (from jackal_description)
├── vicon/jackal3/jackal3 (from vicon_bridge + EKF)
├── vicon/dynamic_object1/dynamic_object1
├── vicon/dynamic_object2/dynamic_object2
├── ...
└── vicon/dingo1/dingo1
```

## Configuration Guide

### Network Configuration

1. **Vicon System IP**: Set in vicon_bridge launch file
```xml
<arg name="datastream_hostport" default="192.168.0.232"/>
```

2. **Object Names**: Must match Vicon Nexus/Tracker object names
```xml
<arg name="object_names" value="[jackal3, dynamic_object1, ...]"/>
```

### Frequency Tuning

1. **Vicon Frequency Divider**: Reduces computational load
```xml
<arg name="object_frequency_divider" default="[2, 2, 2, ...]"/>
```

2. **EKF Frequency**: Configured in ekf.yaml
```yaml
frequency: 50  # Hz
```

### Object Classification

1. **Dynamic Object Radius**: For obstacle avoidance
```xml
<arg name="dynamic_object_radius" default="0.4"/>  <!-- meters -->
```

2. **Static Object Configuration**: For fixed obstacles
```xml
<arg name="static_object_topics" default="[rectangle2x1_1]"/>
<arg name="static_object_sizes" default="[2]"/>  <!-- multiples of radius -->
```

## Troubleshooting

### Common Issues

#### 1. "Could not obtain transform from odom to base_link"
**Cause**: Missing robot description or EKF misconfiguration
**Solution**: Ensure jackal_description is included and EKF base_link_frame is correct

#### 2. "Couldn't find an AF_INET address for [hostname]"
**Cause**: DNS resolution issues with Vicon system
**Solution**: Use IP address instead of hostname in datastream_hostport

#### 3. "use_control is set to true, but control_config is missing"
**Cause**: EKF configuration error
**Solution**: Set `use_control: false` in ekf.yaml or provide complete control configuration

#### 4. Objects not appearing in obstacle message
**Cause**: Object names mismatch between vicon_bridge and vicon_util
**Solution**: Ensure consistent naming in both package configurations

### Diagnostic Topics

Monitor these topics for debugging:

1. **Raw Vicon Data**: `/vicon/{object_name}`
2. **EKF Output**: `/{object_name}/odometry/filtered`
3. **Bundled Objects**: `/vicon_util/dynamic_objects`
4. **TF Tree**: Use `rosrun rqt_tf_tree rqt_tf_tree`

### Log Analysis

Key log messages to monitor:

1. **Vicon Connection**: "Connected to Vicon DataStream"
2. **EKF Status**: "Launched EKFs for [object_list]"
3. **Object Reception**: "Received {n} obstacles"

## Performance Considerations

### Computational Load

1. **EKF Nodes**: Each object requires separate EKF node
   - CPU impact: ~1-2% per object
   - Memory: ~10-20MB per EKF node

2. **Frequency Trade-offs**:
   - Higher frequency = better tracking accuracy
   - Lower frequency = reduced computational load

### Network Bandwidth

1. **Vicon DataStream**: ~1-5 Mbps depending on object count
2. **ROS Topics**: Each pose message ~500 bytes at 50-100 Hz

### Latency Analysis

Typical latency chain:
1. Vicon cameras: ~2-10ms
2. DataStream transmission: ~1-5ms
3. vicon_bridge processing: ~1ms
4. EKF filtering: ~1-2ms
5. Message bundling: ~1ms

**Total system latency**: ~5-20ms

### Optimization Recommendations

1. **Reduce object count** when possible
2. **Increase frequency divider** for non-critical objects
3. **Disable visualization** in production
4. **Use wired network** for Vicon connection
5. **Tune EKF parameters** for specific use case

---

## Conclusion

The Vicon integration provides a robust foundation for multi-robot localization and obstacle detection. The two-stage architecture (vicon_bridge + vicon_util) allows for flexible configuration while maintaining high performance. The system has been designed to handle multiple robots and dynamic objects simultaneously, making it suitable for complex multi-agent scenarios.

For additional information, refer to the individual package documentation and the ROS parameter server during runtime for real-time configuration details.