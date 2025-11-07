#!/usr/bin/env python3
"""
Central Aggregator Publisher Node

This node implements the ZeroMQ publisher component of the central aggregator middleware.
It subscribes to the local ROS topic '/all_robots_reached_objective' and republishes 
the aggregated objective status to the ZeroMQ network for distribution to all robots.

Architecture:
- Listens to: Local ROS topic '/all_robots_reached_objective' (from aggregation logic)
- Publishes to: ZeroMQ network on topic '/all_robots_reached_objective'
- Purpose: Broadcast global objective status to all robots via ZeroMQ

Author: [Jules Zwanen]
Date: November 2025
"""

import rospy
import sys
import os

# Add the scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

from middleware_ros_to_zeroMQ_class import MiddleWareCentralAggBasePub

if __name__ == "__main__":
    middleware_pub = None
    
    try:
        rospy.init_node('middleware_central_agg_pub', log_level=rospy.INFO)
        rospy.loginfo("[Central Aggregator Publisher] Starting central aggregator publisher node")
        
        # Initialize the central aggregator publisher middleware
        middleware_pub = MiddleWareCentralAggBasePub()
        rospy.loginfo("[Central Aggregator Publisher] Central aggregator publisher middleware initialized successfully")
        
        # The MiddleWareCentralAggBasePub class will:
        # 1. Subscribe to local ROS topic '/all_robots_reached_objective'
        # 2. Forward these messages to ZeroMQ network for robot distribution
        # 3. Enable all robots to receive global objective status updates
        
        rospy.loginfo("[Central Aggregator Publisher] Node ready - waiting for objective status messages to relay")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("[Central Aggregator Publisher] Node interrupted by user")
    except Exception as e:
        rospy.logerr(f"[Central Aggregator Publisher] Node failed with error: {e}")
        import traceback
        rospy.logerr(f"[Central Aggregator Publisher] Traceback: {traceback.format_exc()}")
    finally:
        if middleware_pub is not None:
            rospy.loginfo("[Central Aggregator Publisher] Cleaning up middleware resources")
            middleware_pub.cleanup()
            rospy.loginfo("[Central Aggregator Publisher] Cleanup completed")