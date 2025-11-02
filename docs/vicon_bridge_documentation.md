# Vicon Bridge Package Documentation

## Table of Contents
1. [Overview](#overview)
2. [Package Architecture](#package-architecture)
3. [Connection and Data Flow](#connection-and-data-flow)
4. [Message Types and Publishers](#message-types-and-publishers)
5. [Transform Broadcasting](#transform-broadcasting)
6. [Configuration System](#configuration-system)
7. [Object-Specific Configuration](#object-specific-configuration)
8. [Frequency Control](#frequency-control)
9. [Services](#services)
10. [Message Definitions](#message-definitions)
11. [Integration Examples](#integration-examples)
12. [Troubleshooting](#troubleshooting)

## Overview

The `vicon_bridge` package provides a ROS interface to Vicon motion capture systems. It connects to the Vicon DataStream API and publishes real-time pose information for tracked objects (subjects and segments) as ROS messages and TF transforms.

### Key Features
- **Real-time streaming** from Vicon Nexus/Tracker systems
- **Multiple message types** supported (PoseStamped, PoseWithCovarianceStamped, TransformStamped)
- **Flexible object configuration** with per-object settings
- **Transform broadcasting** for robot localization
- **Frequency control** to manage computational load
- **Automatic reconnection** on connection loss
- **Marker data streaming** for detailed analysis

## Package Architecture

### Core Components

```
vicon_bridge/
├── src/
│   ├── vicon_bridge.cpp          # Main node implementation
│   └── calibrate_segment.cpp     # Calibration utilities
├── include/vicon_bridge/
│   ├── vicon_bridge.h            # Main class definition
│   ├── segment_publisher.h       # Base publisher class
│   ├── segment_publisher_posestamped.h
│   ├── segment_publisher_posewithcovariancestamped.h
│   └── segment_publisher_transformstamped.h
├── launch/
│   └── vicon.launch              # Main launch configuration
├── msg/
│   ├── Marker.msg                # Individual marker message
│   └── Markers.msg               # Array of markers
├── srv/
│   ├── viconGrabPose.srv         # Pose grabbing service
│   └── viconCalibrateSegment.srv # Calibration service
└── vicon_sdk/                    # Vicon DataStream SDK
```

### Class Hierarchy

```cpp
ViconReceiver (main class)
├── Client vicon_client_                    // Vicon SDK client
├── tf::TransformBroadcaster tf_broadcaster_ // TF publisher
├── std::map<string, SegmentPublisher> segment_publishers_
└── SegmentPublisher (polymorphic)
    ├── SegmentPublisherPoseStamped
    ├── SegmentPublisherPosewithcovarianceStamped
    └── SegmentPublisherTransformStamped
```

## Connection and Data Flow

### Connection Process

1. **Initialization**:
```cpp
// Connect to Vicon DataStream server
vicon_client_.Connect(host_name);  // e.g., "192.168.0.232:801"
```

2. **Stream Configuration**:
```cpp
// Set streaming mode
vicon_client_.SetStreamMode(StreamMode::ServerPush);  // or ClientPull
// Configure coordinate system (Z-up)
vicon_client_.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
// Enable data streams
vicon_client_.EnableSegmentData();
```

3. **Data Processing Loop**:
```cpp
while (ros::ok()) {
    if (vicon_client_.GetFrame().Result == Result::Success) {
        process_frame();  // Process subjects and markers
    } else {
        // Handle reconnection
        vicon_client_.Disconnect();
        // ... reconnection logic
    }
}
```

### Data Processing Pipeline

```
Vicon System → DataStream API → vicon_bridge → ROS Topics + TF
    ↓               ↓              ↓              ↓
Subjects/        Network      process_frame()   Publishers
Segments         Protocol     process_subjects() + Transforms
Markers                       process_markers()
```

### Frame Processing Details

```cpp
void ViconReceiver::process_frame() {
    // Get frame information
    Output_GetFrameNumber OutputFrameNum = vicon_client_.GetFrameNumber();
    
    // Calculate latency compensation
    ros::Duration vicon_latency(vicon_client_.GetLatencyTotal().Total);
    ros::Time frame_time = ros::Time::now() - vicon_latency;
    
    // Process data
    if(publish_segments_) {
        process_subjects(frame_time);  // Pose data
    }
    if(publish_markers_) {
        process_markers(frame_time, frame_number);  // Marker data
    }
}
```

## Message Types and Publishers

### Supported Message Types

The bridge supports three ROS message types:

#### 1. geometry_msgs/PoseStamped
```cpp
geometry_msgs::PoseStamped pose;
pose.header.stamp = frame_time;
pose.header.frame_id = frame_id;
pose.pose.position.x = position[0];
pose.pose.position.y = position[1];
pose.pose.position.z = position[2] - z_axis_offset;
pose.pose.orientation.x = rotation[0];  // quaternion
pose.pose.orientation.y = rotation[1];
pose.pose.orientation.z = rotation[2];
pose.pose.orientation.w = rotation[3];
```

#### 2. geometry_msgs/PoseWithCovarianceStamped
```cpp
geometry_msgs::PoseWithCovarianceStamped pose_with_cov;
pose_with_cov.header.stamp = frame_time;
pose_with_cov.header.frame_id = frame_id;
pose_with_cov.pose.pose = pose;  // Same as above
// Covariance matrix remains default (zeros)
```

#### 3. geometry_msgs/TransformStamped
```cpp
geometry_msgs::TransformStamped transform;
transform.header.stamp = frame_time;
transform.header.frame_id = parent_frame;
transform.child_frame_id = object_name;
transform.transform.translation.x = position[0];
transform.transform.translation.y = position[1];
transform.transform.translation.z = position[2] - z_axis_offset;
transform.transform.rotation = rotation;  // quaternion
```

### Publisher Creation Strategy

The bridge uses **dynamic publisher creation**:

```cpp
// Publishers are created on-demand when objects are first detected
if (segment_publishers_.find(name) == segment_publishers_.end()) {
    // Create new publisher based on configuration
    if (msg_type == "geometry_msgs/PoseStamped") {
        segment_publishers_.insert(std::make_pair(name, 
            new SegmentPublisherPoseStamped(nh, frame_id, topic_name, frequency_divider, z_offset)));
    }
    // ... other message types
}
```

### Object Naming Convention

Objects are named using the pattern:
- **Single segment**: `subject_name` (e.g., "jackal3")
- **Multiple segments**: `subject_name/segment_name` (e.g., "human/head")

## Transform Broadcasting

### TF Transform System

When `publish_tf=true`, the bridge publishes TF transforms:

```cpp
if (publish_tf_) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(trans.Translation[0]/1000, 
                                   trans.Translation[1]/1000, 
                                   trans.Translation[2]/1000));
    transform.setRotation(tf::Quaternion(quat.Rotation[0], quat.Rotation[1], 
                                        quat.Rotation[2], quat.Rotation[3]));
    
    transforms.push_back(tf::StampedTransform(transform, frame_time, frame_id_all_, name));
}
tf_broadcaster_.sendTransform(transforms);
```

### Transform Tree Structure

```
map (frame_id_all_)
├── jackal3 (from vicon_bridge)
├── dynamic_object1 (from vicon_bridge)
├── dynamic_object2 (from vicon_bridge)
└── ... (other tracked objects)
```

### Coordinate System

The bridge configures Vicon data with **Z-up** coordinate system:
```cpp
vicon_client_.SetAxisMapping(Direction::Forward,  // X forward
                            Direction::Left,     // Y left  
                            Direction::Up);      // Z up
```

**Unit Conversion**: Vicon data (millimeters) → ROS (meters)
```cpp
position[0] = trans.Translation[0] / 1000;  // mm to m
position[1] = trans.Translation[1] / 1000;
position[2] = trans.Translation[2] / 1000;
```

## Configuration System

### Global Configuration Parameters

```xml
<!-- vicon.launch -->
<arg name="datastream_hostport" default="192.168.0.232"/>  <!-- Vicon server IP -->
<arg name="frame_id" default="map"/>                       <!-- Reference frame -->
<arg name="msg_type" default="geometry_msgs/PoseStamped"/> <!-- Default message type -->
<arg name="frequency_divider" default="1"/>                <!-- Global frequency control -->
<arg name="publish_tf" default="true"/>                    <!-- Enable TF broadcasting -->
<arg name="reset_z_axis" default="true"/>                  <!-- Z-axis calibration -->
```

### Stream Mode Configuration

```xml
<param name="stream_mode" value="ServerPush"/>  <!-- or "ClientPull" -->
```

**ServerPush**: Vicon pushes data to client (recommended for real-time)
**ClientPull**: Client requests data from Vicon (more control over timing)

### Z-Axis Reset Feature

When `reset_z_axis=true`, the bridge calibrates the Z-coordinate:

```cpp
if (reset_z_axis_) {
    Output_GetSegmentGlobalTranslation trans = vicon_client_.GetSegmentGlobalTranslation(subject_name, segment_name);
    if (!trans.Occluded) {
        z_axis_offset = trans.Translation[2] / 1000;  // Store initial Z position
    }
}
// Later, during publishing:
pose.pose.position.z = position[2] - z_axis_offset;  // Subtract offset
```

## Object-Specific Configuration

### Configuration Override System

The bridge supports **per-object configuration** via the `object_specific` parameter group:

```xml
<arg name="only_use_object_specific" default="true"/>  <!-- Ignore global settings -->

<arg name="object_names" default="[jackal3, dynamic_object1, dynamic_object2]"/>
<arg name="object_msg_types" default="[geometry_msgs/PoseWithCovarianceStamped, 
                                       geometry_msgs/PoseStamped, 
                                       geometry_msgs/PoseStamped]"/>
<arg name="object_frame_ids" default="[map, map, map]"/>
<arg name="object_publish_topics" default="[/vicon/jackal3, 
                                           /vicon/dynamic_object1, 
                                           /vicon/dynamic_object2]"/>
<arg name="object_frequency_divider" default="[2, 2, 2]"/>
```

### Configuration Storage

Object-specific settings are stored in a map:
```cpp
std::map<std::string, std::array<std::string, 4>> object_specific_details_;
// Key: object_name
// Value: [msg_type, frame_id, publish_topic, frequency_divider]
```

### Publisher Creation Logic

```cpp
// Check for object-specific configuration
auto object_it = object_specific_details_.find(name);

if (object_it != object_specific_details_.end()) {
    // Use object-specific settings
    std::string msg_type = object_it->second[0];
    std::string frame_id = object_it->second[1]; 
    std::string publish_topic = object_it->second[2];
    int frequency_divider = std::stoi(object_it->second[3]);
    
    // Create publisher with specific settings
    segment_publishers_.insert(std::make_pair(name, 
        new SegmentPublisherPosewithcovarianceStamped(nh, frame_id, publish_topic, frequency_divider, z_offset)));
    
    // Remove from map (one-time use)
    object_specific_details_.erase(object_it);
    
} else if (!object_specific_only_) {
    // Use global settings
    segment_publishers_.insert(std::make_pair(name, 
        new SegmentPublisherPoseStamped(nh, frame_id_all_, name, frequency_divider_all_, z_offset)));
}
```

## Frequency Control

### Global Frequency Control

```xml
<arg name="frequency_divider" default="1"/>  <!-- Publish every frame -->
```

### Per-Object Frequency Control

```xml
<arg name="object_frequency_divider" default="[2, 1, 5]"/>
<!-- jackal3: every 2nd frame, dynamic_object1: every frame, dynamic_object2: every 5th frame -->
```

### Implementation

Each publisher maintains its own counter:
```cpp
class SegmentPublisher {
    int counter = 0;
    int publish_on_count = frequency_divider;  // Configurable
    
    virtual void publishMsg(...) {
        counter++;
        if (counter < publish_on_count) {
            return;  // Skip this frame
        }
        counter = 0;  // Reset counter
        
        // Publish message
        pub_.publish(message);
    }
};
```

### Frequency Calculation

If Vicon runs at 100 Hz and `frequency_divider=5`:
- **Effective frequency**: 100 Hz / 5 = 20 Hz
- **Actual behavior**: Publish every 5th frame

## Services

### 1. viconGrabPose Service

**Purpose**: Capture and average multiple pose measurements

**Definition**:
```ros.srv
# Request
string subject_name
string segment_name
int32 n_measurements

---
# Response
bool success
geometry_msgs/PoseStamped pose
```

**Implementation**:
```cpp
bool ViconReceiver::grabPoseCallback(vicon_bridge::viconGrabPose::Request& req, 
                                     vicon_bridge::viconGrabPose::Response& resp) {
    tf::TransformListener tf_listener;
    tf::Quaternion orientation(0, 0, 0, 0);
    tf::Vector3 position(0, 0, 0);
    
    int N = req.n_measurements;
    int n_success = 0;
    
    for (int k = 0; k < N; k++) {
        try {
            tf_listener.lookupTransform(frame_id_all_, tracked_segment, ros::Time(0), transform);
            orientation += transform.getRotation();
            position += transform.getOrigin();
            n_success++;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }
    }
    
    // Average the measurements
    orientation /= n_success;
    orientation.normalize();
    position /= n_success;
    
    // Fill response
    resp.pose.pose.position.x = position.x();
    resp.pose.pose.position.y = position.y();
    resp.pose.pose.position.z = position.z();
    resp.pose.pose.orientation.w = orientation.w();
    resp.pose.pose.orientation.x = orientation.x();
    resp.pose.pose.orientation.y = orientation.y();
    resp.pose.pose.orientation.z = orientation.z();
    
    return true;
}
```

### 2. viconCalibrateSegment Service

**Purpose**: Calibrate object origin relative to Vicon coordinate system

**Definition**:
```ros.srv
# Request
string subject_name
string segment_name
int32 n_measurements
float64 z_offset

---
# Response
bool success
string status
geometry_msgs/PoseStamped pose
```

**Usage**: Used for setting custom coordinate origins for tracked objects.

## Message Definitions

### vicon_bridge/Marker

```ros.msg
string marker_name      # Name of the marker in Vicon
string subject_name     # Parent subject name
string segment_name     # Parent segment name  
geometry_msgs/Point translation  # 3D position in world coordinates
bool occluded          # Whether marker is currently visible
```

### vicon_bridge/Markers

```ros.msg
Header header           # Timestamp and frame info
uint32 frame_number     # Vicon frame number for synchronization
vicon_bridge/Marker[] markers  # Array of all markers (labeled + unlabeled)
```

### Marker Processing

```cpp
void ViconReceiver::process_markers(const ros::Time& frame_time, unsigned int vicon_frame_num) {
    vicon_bridge::Markers markers_msg;
    markers_msg.header.stamp = frame_time;
    markers_msg.frame_number = vicon_frame_num;
    
    // Process labeled markers
    unsigned int SubjectCount = vicon_client_.GetSubjectCount().SubjectCount;
    for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex) {
        std::string subject_name = vicon_client_.GetSubjectName(SubjectIndex).SubjectName;
        unsigned int MarkerCount = vicon_client_.GetMarkerCount(subject_name).MarkerCount;
        
        for (unsigned int MarkerIndex = 0; MarkerIndex < MarkerCount; ++MarkerIndex) {
            vicon_bridge::Marker marker;
            marker.marker_name = vicon_client_.GetMarkerName(subject_name, MarkerIndex).MarkerName;
            marker.subject_name = subject_name;
            
            Output_GetMarkerGlobalTranslation trans = 
                vicon_client_.GetMarkerGlobalTranslation(subject_name, marker.marker_name);
            
            marker.translation.x = trans.Translation[0];
            marker.translation.y = trans.Translation[1]; 
            marker.translation.z = trans.Translation[2];
            marker.occluded = trans.Occluded;
            
            markers_msg.markers.push_back(marker);
        }
    }
    
    // Process unlabeled markers
    unsigned int UnlabeledMarkerCount = vicon_client_.GetUnlabeledMarkerCount().MarkerCount;
    for (unsigned int i = 0; i < UnlabeledMarkerCount; ++i) {
        Output_GetUnlabeledMarkerGlobalTranslation trans = 
            vicon_client_.GetUnlabeledMarkerGlobalTranslation(i);
        
        if (trans.Result == Result::Success) {
            vicon_bridge::Marker marker;
            marker.translation.x = trans.Translation[0];
            marker.translation.y = trans.Translation[1];
            marker.translation.z = trans.Translation[2];
            marker.occluded = false;  // Unlabeled markers can't be occluded
            markers_msg.markers.push_back(marker);
        }
    }
    
    marker_pub_.publish(markers_msg);
}
```

## Integration Examples

### Example 1: Basic Single Robot Tracking

```xml
<include file="$(find vicon_bridge)/launch/vicon.launch">
    <arg name="datastream_hostport" value="192.168.0.232"/>
    <arg name="object_names" value="[jackal3]"/>
    <arg name="object_msg_types" value="[geometry_msgs/PoseWithCovarianceStamped]"/>
    <arg name="object_frame_ids" value="[map]"/>
    <arg name="object_publish_topics" value="[/vicon/jackal3]"/>
    <arg name="object_frequency_divider" value="[2]"/>
</include>
```

**Result**:
- **Topic**: `/vicon/jackal3` (PoseWithCovarianceStamped at 50 Hz if Vicon runs at 100 Hz)
- **TF**: `map` → `jackal3` transform
- **Frame**: `map`

### Example 2: Multi-Robot + Dynamic Objects

```xml
<include file="$(find vicon_bridge)/launch/vicon.launch">
    <arg name="object_names" value="[jackal3, jackal4, dynamic_object1, dynamic_object2]"/>
    <arg name="object_msg_types" value="[geometry_msgs/PoseWithCovarianceStamped, 
                                        geometry_msgs/PoseWithCovarianceStamped,
                                        geometry_msgs/PoseStamped,
                                        geometry_msgs/PoseStamped]"/>
    <arg name="object_frame_ids" value="[map, map, map, map]"/>
    <arg name="object_publish_topics" value="[/vicon/jackal3, /vicon/jackal4, 
                                             /vicon/dynamic_object1, /vicon/dynamic_object2]"/>
    <arg name="object_frequency_divider" value="[2, 2, 5, 5]"/>
</include>
```

**Result**:
- **Robot topics**: `/vicon/jackal3`, `/vicon/jackal4` (PoseWithCovarianceStamped at 50 Hz)
- **Object topics**: `/vicon/dynamic_object1`, `/vicon/dynamic_object2` (PoseStamped at 20 Hz)
- **TF transforms**: All objects published to TF tree

### Example 3: Integration with ros1_jackal.launch

```xml
<!-- From ros1_jackal.launch -->
<include file="$(find vicon_bridge)/launch/vicon.launch">
    <arg name="object_names" value="[$(arg jackal_name), dynamic_object1, dynamic_object2, 
                                     dynamic_object3, dynamic_object4, dynamic_object5, dingo1]"/>
    <arg name="object_msg_types" default="[geometry_msgs/PoseWithCovarianceStamped, 
                                          geometry_msgs/PoseWithCovarianceStamped,
                                          geometry_msgs/PoseWithCovarianceStamped, 
                                          geometry_msgs/PoseWithCovarianceStamped,
                                          geometry_msgs/PoseWithCovarianceStamped, 
                                          geometry_msgs/PoseWithCovarianceStamped,
                                          geometry_msgs/PoseWithCovarianceStamped]"/>
    <arg name="object_frame_ids" default="[map, map, map, map, map, map, map]"/>
    <arg name="object_publish_topics" default="[/vicon/$(arg jackal_name), /vicon/dynamic_object1,
                                               /vicon/dynamic_object2, /vicon/dynamic_object3, 
                                               /vicon/dynamic_object4, /vicon/dynamic_object5, 
                                               /vicon/dingo1]"/>
    <arg name="object_frequency_divider" default="[2, 2, 2, 2, 2, 2, 2]"/>
</include>
```

**Integration Flow**:
1. **vicon_bridge** publishes raw poses: `/vicon/jackal3`, `/vicon/dynamic_object1`, etc.
2. **vicon_util EKFs** subscribe to these topics and add velocity estimation
3. **EKF output**: `/jackal3/odometry/filtered`, `/dynamic_object1/odometry/filtered`, etc.
4. **obstacle_bundle_node** aggregates all objects into `/vicon_util/dynamic_objects`
5. **MPC planner** uses bundled obstacles for navigation planning

## Troubleshooting

### Common Issues

#### 1. Connection Problems

**Error**: "Error while connecting to Vicon. Exiting now."
**Causes**:
- Incorrect IP address/hostname
- Vicon DataStream not running
- Network connectivity issues
- Firewall blocking port 801

**Solutions**:
```bash
# Test network connectivity
ping 192.168.0.232

# Check if DataStream is running on Vicon computer
telnet 192.168.0.232 801

# Update launch file with correct IP
<arg name="datastream_hostport" default="192.168.0.232:801"/>
```

#### 2. No Objects Detected

**Error**: No topics published, no objects appearing
**Causes**:
- Objects not tracked in Vicon Nexus/Tracker
- Object names mismatch between Vicon and ROS configuration
- Objects occluded during initialization

**Solutions**:
```bash
# Check what objects Vicon is tracking
rostopic echo /vicon/markers

# Verify object names in Vicon match configuration
# Check Vicon Nexus "Objects" panel for exact names

# Enable debug output
roslaunch vicon_bridge vicon.launch --screen
```

#### 3. Transform Issues

**Error**: "Could not transform from vicon/object to map"
**Causes**:
- `publish_tf=false` but transforms expected
- Frame ID mismatches
- TF buffer not populated

**Solutions**:
```bash
# Check TF tree
rosrun rqt_tf_tree rqt_tf_tree

# Verify frame IDs
rostopic echo /vicon/jackal3 | grep frame_id

# Enable transform publishing
<arg name="publish_tf" default="true"/>
```

#### 4. Frequency/Performance Issues

**Error**: High CPU usage, dropped frames
**Causes**:
- Too many objects tracked at high frequency
- Insufficient computational resources
- Network bandwidth limitations

**Solutions**:
```xml
<!-- Reduce publishing frequency -->
<arg name="object_frequency_divider" default="[5, 10, 10, 10]"/>

<!-- Disable unnecessary features -->
<param name="publish_markers" value="false"/>

<!-- Use ClientPull mode for better control -->
<param name="stream_mode" value="ClientPull"/>
```

#### 5. Coordinate System Issues

**Error**: Objects appearing in wrong locations/orientations
**Causes**:
- Coordinate system mismatch
- Z-axis calibration issues
- Unit conversion problems

**Solutions**:
```xml
<!-- Disable Z-axis reset if causing issues -->
<arg name="reset_z_axis" default="false"/>

<!-- Check coordinate system configuration in Vicon -->
<!-- Ensure Vicon is configured for Z-up coordinate system -->
```

### Diagnostic Commands

```bash
# Monitor connection status
rostopic echo /rosout | grep vicon

# Check published topics
rostopic list | grep vicon

# Monitor message rates
rostopic hz /vicon/jackal3

# View TF tree
rosrun tf2_tools view_frames.py

# Debug transform lookups
rosrun tf tf_monitor

# Check parameter configuration
rosparam list | grep vicon
rosparam get /vicon/object_specific/object_names
```

### Log Analysis

**Key log messages to monitor**:

1. **Successful connection**:
```
[INFO] [timestamp]: Connecting to Vicon DataStream SDK at 192.168.0.232 ...
[INFO] [timestamp]: ... connected!
[INFO] [timestamp]: Vicon client framerate: 100.000000
```

2. **Object detection**:
```
[INFO] [timestamp]: creating new object jackal3 ...
[INFO] [timestamp]: ... done, advertised as jackal3
[INFO] [timestamp]: Object jackal3: 
                     msg type: geometry_msgs/PoseWithCovarianceStamped 
                     frame id: map 
                     topic name: /vicon/jackal3 
                     frequency divider: 2 
                     actual frequency: 50
```

3. **Occlusion warnings**:
```
[WARN] [timestamp]: jackal3 occluded, not publishing...
```

4. **Connection loss/recovery**:
```
[WARN] [timestamp]: Vicon client connection lost. Waiting for connection to grab a new frame ...
[INFO] [timestamp]: ... connection re-established!
```

---

## Conclusion

The `vicon_bridge` package provides a robust and flexible interface between Vicon motion capture systems and ROS. Its key strengths include:

- **Flexible configuration** supporting multiple objects with individual settings
- **Multiple message types** for different use cases
- **Automatic object discovery** and dynamic publisher creation
- **Robust connection handling** with automatic reconnection
- **Performance optimization** through frequency control
- **Integration-ready** design for complex multi-robot systems

The package forms the foundation for precise localization in the multi-robot navigation system, providing the raw pose data that gets enhanced by vicon_util for velocity estimation and obstacle bundling.

For integration with the larger system, the vicon_bridge serves as the first stage in the data pipeline:
**Vicon Hardware** → **vicon_bridge** → **vicon_util** → **MPC Planner**

This architecture ensures clean separation of concerns while maintaining high performance and reliability for real-time robotic applications.