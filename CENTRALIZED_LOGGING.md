# Centralized Logging System for Middleware Nodes

## Overview

The middleware logging has been refactored to eliminate console spam by implementing a centralized logging buffer system. Instead of logging each initialization step individually, all startup information is collected and displayed **once** in a clean, organized summary format.

## Key Changes

### 1. New `MiddlewareLogger` Class (`middleware_logger.py`)

A centralized logger that:
- **Buffers** all initialization logs during startup
- **Organizes** information into logical sections (Config, Connections, Subscriptions, Publications)
- **Displays** everything in a single, formatted summary at the end of initialization
- **Provides** runtime event logging with consistent formatting

### 2. Refactored Classes

All middleware classes now use the centralized logger:

- **MiddleWarePublisher** - ROS → ZeroMQ Publisher
- **MiddleWareSubscriber** - ZeroMQ → ROS Subscriber  
- **MiddleWareCentralAggBasePub** - Central Aggregator Publisher
- **MiddleWareCentralAggBaseSub** - Central Aggregator Subscriber

### 3. Node Startup Files

Simplified all node startup files to remove redundant logging:
- `middleware_publisher_node.py`
- `middleware_subscriber_node.py`
- `middleware_ros_to_zermoMQ_central_agg_pub.py`
- `middleware_zermoMQ_to_ros_central_agg_sub.py`

## Before vs After

### Before (Verbose Console Spam)
```
[ INFO] [1702915234.123]: /jackal1: [ROS → ZeroMQ Publisher] Extracted robot ID: 1
[ INFO] [1702915234.124]: /jackal1: [ROS → ZeroMQ Publisher] Found 2 other robots: ['/jackal2', '/jackal3']
[ INFO] [1702915234.125]: /jackal1: [ROS → ZeroMQ Publisher] Successfully bound to tcp://192.168.0.101:5001
[ INFO] [1702915234.126]: /jackal1: [ROS → ZeroMQ Publisher] Middleware initialized successfully
...
(15+ more lines per node)
```

### After (Clean Summary)
```
================================================================================
  ROS → ZeroMQ Publisher - Initialization Summary
  Robot: /jackal1
================================================================================

[Configuration]
  • Robot ID: 1
  • Other robots: 2: /jackal2, /jackal3
  • Publisher endpoint: tcp://192.168.0.101:5001

[ZeroMQ Connections]
  • tcp://192.168.0.101:5001 (Bound)

[Subscriptions] (2 topics)
  • /jackal1/robot_to_robot/output/current_trajectory [ObstacleGMM]
  • /jackal1/events/objective_reached [Bool]

--------------------------------------------------------------------------------
  Status: READY ✓
================================================================================
```

## Usage

### In Classes (Initialization)

```python
class MiddleWarePublisher:
    def __init__(self):
        # Initialize logger
        self.logger = MiddlewareLogger("ROS → ZeroMQ Publisher", self._ego_robot_ns)
        
        # Buffer configuration info
        self.logger.add_config("Robot ID", self._ego_robot_id)
        self.logger.add_config("Other robots", f"{len(self._other_robots_nss)} robots")
        
        # Buffer connections
        self.logger.add_connection(endpoint, "Bound")
        
        # Buffer subscriptions/publications
        self.logger.add_subscription(topic, "ObstacleGMM")
        
        # Buffer warnings/errors
        self.logger.add_warning("Using default endpoint")
        self.logger.add_error("Failed to connect")
        
        # Display everything at the end
        self.logger.log_startup_summary()
```

### In Callbacks (Runtime Events)

```python
def trajectory_callback(self, msg):
    try:
        # Process message...
        rospy.logdebug(f"{self._ego_robot_ns}: [ROS → ZeroMQ] Relayed trajectory seq={self.seq}")
    except Exception as e:
        # Use centralized logging for errors
        self.logger.log_runtime_event("ROS → ZeroMQ", f"Failed to relay: {e}", "error")
```

## Benefits

1. **Reduced Console Spam**: ~15 lines per node → 1 clean summary block
2. **Better Organization**: All related info grouped by category
3. **Easier Debugging**: Status indicator shows errors/warnings at a glance
4. **Consistent Formatting**: All nodes use same structured output
5. **Single Point of Control**: Change logging format in one place

## Testing

Run the test script to see examples:

```bash
source devel/setup.bash
rosrun guidance_planner_multi_robot_nodes test_middleware_logging.py
```

This demonstrates the new logging format for all middleware components.

## Migration Notes

- All initialization logging now goes through `logger.add_*()` methods
- `log_startup_summary()` must be called at end of `__init__()`
- Runtime events use `logger.log_runtime_event()` for consistency
- Debug messages can still use `rospy.logdebug()` directly (low overhead)
- Error messages should use `logger.log_runtime_event(..., "error")` for formatting

## Future Improvements

Consider adding:
- Log level filtering (show only warnings/errors in summary)
- Timing information (initialization duration)
- Network diagnostics (latency, bandwidth)
- Configuration validation warnings
