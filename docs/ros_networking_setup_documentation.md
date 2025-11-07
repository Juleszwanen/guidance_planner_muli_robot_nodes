# ROS Networking Setup Documentation

## Table of Contents
1. [Overview](#overview)
2. [The Problem We Solved](#the-problem-we-solved)
3. [Understanding ROS Networking](#understanding-ros-networking)
4. [Our Solution Explained](#our-solution-explained)
5. [Component Analysis](#component-analysis)
6. [Bidirectional Communication Flow](#bidirectional-communication-flow)
7. [Adapting for Different Jackals](#adapting-for-different-jackals)
8. [Troubleshooting Guide](#troubleshooting-guide)
9. [Advanced Networking Concepts](#advanced-networking-concepts)

## Overview

This documentation explains the ROS networking configuration required for communication between a Docker container development environment and physical Jackal robots. The setup enables bidirectional communication for topics, services, and transforms across different network nodes.

### Key Components
- **Docker Container**: Development environment at `192.168.0.99`
- **Jackal Robots**: Physical robots with hostnames like `cpr-j100-0114`
- **ROS Master**: Distributed system coordinator
- **Network Configuration**: Hostname resolution and IP mapping

## The Problem We Solved

### Initial Error Symptoms
```bash
Couldn't find an AF_INET address for [cpr-j100-0114]: Temporary failure in name resolution
Error in XmlRpcClient::doConnect: Could not connect to server (Resource temporarily unavailable)
```

### Root Cause Analysis

#### **Problem 1: Hostname Resolution Failure**
```
Docker Container (192.168.0.99)
├── /etc/hosts: Only localhost entries
├── DNS: Cannot resolve cpr-j100-0114
└── Result: Cannot connect to Jackal nodes
```

#### **Problem 2: ROS Master Location**
```
ROS Master Location: Where is the coordinator?
├── Container: No ROS Master running
├── Jackal: ROS Master at localhost:11311
└── Solution: Point container to Jackal's ROS Master
```

#### **Problem 3: Bidirectional Communication**
```
Communication Requirements:
├── Container → Jackal: Subscribe to joystick topics
├── Jackal → Container: Connect to planning nodes  
└── Both need correct IP resolution
```

## Understanding ROS Networking

### ROS Master Architecture

```
ROS Master (Jackal Robot)
├── IP: 192.168.0.101:11311
├── Role: Service discovery and coordination
├── Maintains: Registry of all nodes, topics, services
└── Enables: Node-to-node communication setup
```

**ROS Master Responsibilities:**
1. **Service Discovery**: "Who publishes `/bluetooth_teleop/joy`?"
2. **Connection Brokering**: "Container, connect to `http://cpr-j100-0114:39321/`"
3. **Parameter Server**: Centralized configuration storage
4. **Node Registration**: Track all active nodes

### Node Communication Flow

```
Step 1: Container connects to ROS Master
Container → ROS Master: "I want to subscribe to /bluetooth_teleop/joy"

Step 2: ROS Master provides publisher info  
ROS Master → Container: "Publisher is at http://cpr-j100-0114:39321/"

Step 3: Direct node connection attempt
Container → cpr-j100-0114:39321: DIRECT CONNECTION (THIS FAILED!)

Step 4: Bidirectional data flow
cpr-j100-0114:39321 ↔ Container: Topic data exchange
```

### Why Hostname Resolution is Critical

**ROS Networking Principle**: After initial service discovery through ROS Master, **nodes communicate directly** with each other using the advertised addresses.

```cpp
// What happens in ROS node communication:
// 1. Node advertises topic with its hostname
publisher.advertise("http://cpr-j100-0114:39321/topic");

// 2. ROS Master stores this address
master.registerPublisher("http://cpr-j100-0114:39321/topic");

// 3. Subscriber gets this address from master
subscriber_address = master.lookupPublisher("/topic");

// 4. Subscriber tries to connect DIRECTLY to publisher
connection = connect(subscriber_address); // <- FAILS without hostname resolution
```

## Our Solution Explained

### Solution Component 1: ROS_MASTER_URI

```bash
export ROS_MASTER_URI=http://192.168.0.101:11311
```

**What this does:**
- Points your container's ROS nodes to the Jackal's ROS Master
- Enables service discovery and topic registration
- Allows container nodes to register themselves with the distributed system

**Why this specific format:**
- `http://`: ROS Master uses XML-RPC over HTTP
- `192.168.0.101`: Jackal 1's IP address on lab network
- `11311`: Default ROS Master port

**Alternative (Wrong) approaches:**
```bash
# WRONG: Points to non-existent local master
export ROS_MASTER_URI=http://localhost:11311

# WRONG: Uses hostname that can't be resolved
export ROS_MASTER_URI=http://cpr-j100-0114:11311
```

### Solution Component 2: ROS_IP

```bash
export ROS_IP=192.168.0.99
```

**What this does:**
- Tells ROS nodes running in your container how to advertise themselves
- Ensures other nodes can connect back to your container
- Overrides automatic IP detection (which might pick Docker's internal IP)

**Why this specific IP:**
- `192.168.0.99`: Your development machine's IP on the lab network
- Accessible from Jackal robots on the same network segment
- Bypasses Docker's internal networking

**What happens without ROS_IP:**
```bash
# Without ROS_IP set:
Container Node advertises: http://172.17.0.2:12345/topic  # Docker internal IP
Jackal tries to connect: FAILURE - 172.17.0.2 not reachable from outside

# With ROS_IP set:
Container Node advertises: http://192.168.0.99:12345/topic  # Lab network IP  
Jackal tries to connect: SUCCESS - 192.168.0.99 reachable
```

### Solution Component 3: Hostname Resolution

```bash
# Added to /etc/hosts in container:
192.168.0.101 cpr-j100-0114
```

**What this does:**
- Maps the Jackal's hostname to its IP address
- Enables direct node-to-node connections
- Resolves DNS lookup failures

**The resolution process:**
```bash
# When ROS node tries to connect to cpr-j100-0114:
1. getent hosts cpr-j100-0114
2. Check /etc/hosts: 192.168.0.101 cpr-j100-0114
3. Return IP: 192.168.0.101
4. Connect to: 192.168.0.101:39321 ✅ SUCCESS
```

## Component Analysis

### jules_connect_to_jackal.sh Script Breakdown

#### **Input Validation**
```bash
if [ $# -eq 0 ]; then
    echo "Usage: source jules_connect_to_jackal.sh <jackal_name>"
    return 1
fi
```
**Purpose**: Ensures user specifies which Jackal to connect to, preventing accidental connections.

#### **Hostname Update Function**
```bash
update_hostname() {
    local ip=$1
    local hostname="cpr-j100-0114"
    
    # Remove existing entry (prevents duplicates)
    sudo sed -i "/$hostname/d" /etc/hosts 2>/dev/null
    
    # Add new mapping
    echo "$ip $hostname" | sudo tee -a /etc/hosts > /dev/null
}
```

**Why this approach:**
- **Idempotent**: Can be run multiple times safely
- **Atomic**: Removes old entry before adding new one
- **Clean**: No duplicate entries in /etc/hosts

#### **Jackal-Specific Configuration**
```bash
case $JACKAL_NAME in
    jackal1)
        export ROS_MASTER_URI=http://192.168.0.101:11311
        update_hostname "192.168.0.101"
        ;;
    jackal2)
        export ROS_MASTER_URI=http://192.168.0.102:11311
        update_hostname "192.168.0.102"
        ;;
esac
```

**Design rationale:**
- **Scalable**: Easy to add new Jackals
- **Consistent**: Same pattern for all robots
- **Maintainable**: Clear mapping between names and IPs

#### **Verification Step**
```bash
if ping -c 1 cpr-j100-0114 > /dev/null 2>&1; then
    echo "✅ Hostname resolution working"
else
    echo "❌ Hostname resolution failed"
fi
```

**Purpose**: Immediate feedback on whether the setup worked correctly.

## Bidirectional Communication Flow

### Container → Jackal Communication

**Scenario**: Container subscribes to joystick topic

```
1. Container Node Startup:
   └── ROS_MASTER_URI → Connect to http://192.168.0.101:11311

2. Topic Subscription:
   Container → ROS Master: "Subscribe to /bluetooth_teleop/joy"
   ROS Master → Container: "Publisher at http://cpr-j100-0114:39321/"

3. Direct Connection:
   Container resolves cpr-j100-0114 → 192.168.0.101 (via /etc/hosts)
   Container → 192.168.0.101:39321: Establish TCP connection

4. Data Flow:
   Jackal Joy Node → Container: Joystick data stream
```

### Jackal → Container Communication

**Scenario**: Jackal connects to container's planning node

```
1. Container Node Advertisement:
   Container → ROS Master: "Publishing /planned_trajectory at http://192.168.0.99:45123/"
   (Uses ROS_IP for correct address advertisement)

2. Jackal Node Discovery:
   Jackal Node → ROS Master: "Who publishes /planned_trajectory?"
   ROS Master → Jackal: "Publisher at http://192.168.0.99:45123/"

3. Direct Connection:
   Jackal → 192.168.0.99:45123: Establish TCP connection
   (No hostname resolution needed - direct IP)

4. Data Flow:
   Container Planning Node → Jackal: Trajectory commands
```

### Network Topology Diagram

```
Lab Network (192.168.0.x)
│
├── Development Machine (192.168.0.99)
│   └── Docker Container (jzwanen)
│       ├── ROS_IP: 192.168.0.99
│       ├── ROS_MASTER_URI: http://192.168.0.101:11311  
│       ├── /etc/hosts: 192.168.0.101 cpr-j100-0114
│       └── Nodes: Planning, Visualization, etc.
│
├── Jackal 1 (192.168.0.101)
│   ├── Hostname: cpr-j100-0114
│   ├── ROS Master: localhost:11311
│   ├── Nodes: Joy, Motors, Sensors
│   └── /etc/hosts: 127.0.1.1 cpr-j100-0114
│
├── Jackal 2 (192.168.0.102)
├── Jackal 3 (192.168.0.103)
└── Jackal 4 (192.168.0.104)
```

## Adapting for Different Jackals

### Scenario: Adding a New Jackal

**If you get a new Jackal with different specifications:**

#### **Step 1: Determine Jackal's Network Identity**
```bash
# SSH into the new Jackal
ssh administrator@192.168.0.105  # New IP

# Check hostname and ROS configuration
hostname                         # e.g., cpr-j100-0115
echo $ROS_ROBOT_SERIAL_NO        # e.g., J100-0115
cat /etc/hosts | grep $(hostname)
```

#### **Step 2: Update Connection Script**
```bash
# Add new case to jules_connect_to_jackal.sh
jackal5)
    export ROS_MASTER_URI=http://192.168.0.105:11311
    echo "Connected to Jackal 5: $ROS_MASTER_URI"
    update_hostname "192.168.0.105"
    ;;
```

#### **Step 3: Handle Different Hostnames**

**Case A: Different hostname pattern**
```bash
# If new Jackal has hostname: cpr-j100-0115
update_hostname() {
    local ip=$1
    local hostname="cpr-j100-0115"  # Updated hostname
    
    sudo sed -i "/$hostname/d" /etc/hosts 2>/dev/null
    echo "$ip $hostname" | sudo tee -a /etc/hosts > /dev/null
}
```

**Case B: Multiple hostnames (Advanced)**
```bash
# For handling multiple different hostnames
update_hostname() {
    local ip=$1
    local hostname=$2  # Pass hostname as parameter
    
    sudo sed -i "/$hostname/d" /etc/hosts 2>/dev/null
    echo "$ip $hostname" | sudo tee -a /etc/hosts > /dev/null
    echo "Updated /etc/hosts: $ip -> $hostname"
}

# Usage in case statement:
jackal5)
    export ROS_MASTER_URI=http://192.168.0.105:11311
    update_hostname "192.168.0.105" "cpr-j100-0115"
    ;;
```

### Different Network Configurations

#### **Different IP Ranges**
```bash
# If Jackals use different network (e.g., 10.0.0.x)
jackal_remote)
    export ROS_MASTER_URI=http://10.0.0.50:11311
    update_hostname "10.0.0.50"
    # Also update ROS_IP if your machine has different IP on that network
    export ROS_IP=10.0.0.99
    ;;
```

#### **Different ROS Master Ports**
```bash
# If ROS Master runs on non-standard port
jackal_custom)
    export ROS_MASTER_URI=http://192.168.0.101:11322  # Custom port
    update_hostname "192.168.0.101"
    ;;
```

### Discovery Script for New Jackals

**Create a helper script to identify new Jackal configurations:**
```bash
#!/bin/bash
# discover_jackal.sh - Helper to identify Jackal network config

JACKAL_IP=$1
if [ -z "$JACKAL_IP" ]; then
    echo "Usage: discover_jackal.sh <jackal_ip>"
    exit 1
fi

echo "Discovering Jackal configuration at $JACKAL_IP..."

# Test connectivity
if ! curl -s --connect-timeout 3 "http://$JACKAL_IP:11311/" > /dev/null; then
    echo "❌ Cannot reach ROS Master at $JACKAL_IP:11311"
    exit 1
fi

# SSH and get info
ssh administrator@$JACKAL_IP << 'EOF'
    echo "Hostname: $(hostname)"
    echo "Serial: $ROS_ROBOT_SERIAL_NO"
    echo "ROS Master: $ROS_MASTER_URI"
    echo "IP Address: $(hostname -I | awk '{print $1}')"
    echo "/etc/hosts entry: $(grep $(hostname) /etc/hosts)"
EOF

echo ""
echo "Add this to your connection script:"
echo "jackal_new)"
echo "    export ROS_MASTER_URI=http://$JACKAL_IP:11311"
echo "    update_hostname \"$JACKAL_IP\""
echo "    ;;"
```

## Troubleshooting Guide

### Common Issues and Solutions

#### **Issue 1: "Could not connect to server"**

**Symptoms:**
```bash
Error in XmlRpcClient::doConnect: Could not connect to server
```

**Diagnosis:**
```bash
# Test ROS Master connectivity
curl -s http://192.168.0.101:11311/

# Check ROS_MASTER_URI
echo $ROS_MASTER_URI

# Test network connectivity  
ping 192.168.0.101
```

**Solutions:**
1. **Wrong IP**: Verify Jackal IP address
2. **ROS Master down**: Check if Jackal is powered on
3. **Network issues**: Verify lab network connectivity

#### **Issue 2: "Hostname resolution failed"**

**Symptoms:**
```bash
Couldn't find an AF_INET address for [cpr-j100-0114]
```

**Diagnosis:**
```bash
# Check /etc/hosts entry
grep cpr-j100-0114 /etc/hosts

# Test resolution
getent hosts cpr-j100-0114

# Verify ping works (after installing iputils-ping)
ping -c 1 cpr-j100-0114
```

**Solutions:**
1. **Missing entry**: Re-run connection script
2. **Wrong IP**: Update /etc/hosts manually
3. **Permission issues**: Ensure sudo access for script

#### **Issue 3: "No topics visible"**

**Symptoms:**
```bash
rostopic list  # Returns empty or minimal list
```

**Diagnosis:**
```bash
# Check ROS environment
echo $ROS_MASTER_URI
echo $ROS_IP

# Test ROS Master connection
rostopic list

# Check network configuration
ip route show
```

**Solutions:**
1. **Wrong ROS_MASTER_URI**: Verify Jackal IP and port
2. **ROS_IP not set**: Ensure container IP is accessible
3. **Firewall issues**: Check Docker/host firewall settings

#### **Issue 4: Container nodes not reachable from Jackal**

**Symptoms:**
- Jackal can't connect to container's published topics
- One-way communication only

**Diagnosis:**
```bash
# Check what IP your nodes advertise
rostopic info /your_topic

# Verify ROS_IP setting
echo $ROS_IP

# Test accessibility from outside container
# (From host machine or another computer)
curl http://192.168.0.99:PORT/
```

**Solutions:**
1. **ROS_IP not set**: Export correct IP address
2. **Docker networking**: Ensure ports are accessible
3. **Wrong IP**: Use lab network IP, not Docker internal IP

### Network Diagnostic Commands

#### **ROS-Specific Diagnostics**
```bash
# List all nodes and their locations
rosnode list
rosnode info /node_name

# Check topic publishers/subscribers
rostopic info /topic_name

# Monitor ROS network traffic
rostopic echo /rosout

# Check parameter server
rosparam list
rosparam get /run_id
```

#### **Network-Level Diagnostics**
```bash
# Check your container's network interfaces
ip addr show

# Check routing table  
ip route show

# Test specific port connectivity
nc -zv 192.168.0.101 11311

# Monitor network connections
netstat -tuln | grep 11311
```

#### **DNS and Hostname Diagnostics**
```bash
# Test hostname resolution methods
nslookup cpr-j100-0114
dig cpr-j100-0114
getent hosts cpr-j100-0114

# Check /etc/hosts file
cat /etc/hosts | grep -v "^#"

# Test ping (after installation)
ping -c 3 cpr-j100-0114
```

## Advanced Networking Concepts

### ROS Network Security Considerations

#### **Trust Model**
```bash
# ROS assumes trusted network environment
# All nodes can communicate with all other nodes
# No authentication or encryption by default
```

**Implications for multi-robot systems:**
- Any node can subscribe to any topic
- Parameter server is accessible to all nodes
- Service calls are unencrypted

#### **Network Isolation**
```bash
# Different ROS networks by Master URI
export ROS_MASTER_URI=http://robot1:11311  # Network 1
export ROS_MASTER_URI=http://robot2:11311  # Network 2 (isolated)
```

### Performance Optimization

#### **Network Bandwidth Management**
```bash
# Large topics can saturate network
rostopic hz /camera/image_raw    # Check frequency
rostopic bw /camera/image_raw    # Check bandwidth usage

# Consider using compressed topics
/camera/image_raw/compressed
/camera/image_raw/theora
```

#### **Latency Optimization**
```bash
# Minimize hops in communication
Container → ROS Master → Direct to Publisher
# vs
Container → Proxy → ROS Master → Proxy → Publisher

# Use appropriate message queue sizes
publisher = rospy.Publisher('/topic', Message, queue_size=10)
```

### Docker Networking Integration

#### **Host Network Mode**
```bash
# Run container with host networking (advanced)
docker run --network=host <image>
# Pros: No IP translation needed
# Cons: No network isolation
```

#### **Bridge Network with Port Mapping**
```bash
# Current approach - bridge network with ROS_IP
# Pros: Network isolation maintained
# Cons: Requires careful IP management
```

#### **Custom Docker Networks**
```bash
# Create dedicated network for robotics
docker network create --subnet=192.168.100.0/24 robotics_net
# Pros: Controlled IP assignment
# Cons: More complex setup
```

## Conclusion

The ROS networking setup we implemented solves the fundamental challenge of enabling bidirectional communication between a containerized development environment and physical robots with hostname-based addressing.

### Key Success Factors

1. **Correct ROS Master Configuration**: Pointing to the robot's ROS Master enables service discovery
2. **Proper IP Advertisement**: ROS_IP ensures container nodes are reachable from external systems  
3. **Hostname Resolution**: /etc/hosts mapping enables direct node-to-node connections
4. **Idempotent Setup**: Script can be run repeatedly without causing issues

### System Benefits

- **Seamless Development**: Develop in container, test on real robots
- **Network Transparency**: Container appears as another node on the robot network
- **Scalable Architecture**: Easy to add new robots or development machines
- **Debugging Capability**: Full access to robot topics and services from development environment

This networking foundation enables complex multi-robot applications while maintaining the flexibility and isolation benefits of containerized development.

---

## Quick Reference

### Essential Commands
```bash
# Connect to Jackal
source connect_to_jackals/jules_connect_to_jackal.sh jackal1

# Verify setup
echo $ROS_MASTER_URI
echo $ROS_IP  
grep cpr-j100-0114 /etc/hosts
rostopic list

# Test connectivity
ping -c 1 cpr-j100-0114
rostopic echo /bluetooth_teleop/joy -n 1
```

### Configuration Variables
```bash
ROS_MASTER_URI=http://192.168.0.101:11311  # Jackal's ROS Master
ROS_IP=192.168.0.99                        # Your container's external IP
/etc/hosts: 192.168.0.101 cpr-j100-0114   # Hostname resolution
```

### Network Topology
```
Container (192.168.0.99) ↔ ROS Master (192.168.0.101) ↔ Jackal Nodes
```

This setup enables full bidirectional ROS communication while maintaining network security and development flexibility.