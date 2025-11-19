#!/usr/bin/env python3
"""
Central Aggregator Subscriber Node

This node implements the ZeroMQ subscriber component of the central aggregator middleware.
It subscribes to individual robot objective reached messages via ZeroMQ and republishes 
them to local ROS topics for further processing by aggregation logic.

Architecture:
- Subscribes to: ZeroMQ messages from individual robots on '/{robotX}/events/objective_reached'
- Publishes to: Local ROS topics '/{robotX}/events/objective_reached' 
- Purpose: Relay individual robot objective status from ZeroMQ network to local ROS ecosystem

Flow:
Robot1 ──ZeroMQ──┐
Robot2 ──ZeroMQ──┤──► Central Agg ──ROS Topics──► Separate Aggregation Node
Robot3 ──ZeroMQ──┘     Subscriber    (Relay)      (Business Logic)

Author: [Your Name]
Date: November 2025
"""

import rospy
import sys
import os

# Add the scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

from middleware_ros_to_zeroMQ_class import MiddleWareCentralAggBaseSub

if __name__ == "__main__":
    middleware_sub = None
    
    try:
        rospy.init_node('middleware_central_agg_sub', log_level=rospy.INFO)
        rospy.loginfo("[Central Aggregator Subscriber] Starting central aggregator subscriber node")
        
        # Initialize the central aggregator subscriber middleware
        middleware_sub = MiddleWareCentralAggBaseSub()
        rospy.loginfo("[Central Aggregator Subscriber] Central aggregator subscriber middleware initialized successfully")
        
        # The MiddleWareCentralAggBaseSub class will:
        # 1. Subscribe to ZeroMQ messages from individual robots on their objective topics
        # 2. Relay these messages to local ROS topics for each robot
        # 3. Enable local aggregation logic to process all robot statuses
        
        rospy.loginfo("[Central Aggregator Subscriber] Node ready - listening for robot objective messages")
        
        # Run message processing loop in main thread (blocking call)
        middleware_sub.message_processing_loop()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("[Central Aggregator Subscriber] Node interrupted by user")
    except Exception as e:
        rospy.logerr(f"[Central Aggregator Subscriber] Node failed with error: {e}")
        import traceback
        rospy.logerr(f"[Central Aggregator Subscriber] Traceback: {traceback.format_exc()}")
    finally:
        if middleware_sub is not None:
            rospy.loginfo("[Central Aggregator Subscriber] Cleaning up middleware resources")
            middleware_sub.cleanup()
            rospy.loginfo("[Central Aggregator Subscriber] Cleanup completed")