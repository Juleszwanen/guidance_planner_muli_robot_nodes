#!/usr/bin/env python3
"""
Test script to demonstrate the centralized logging system.
Run this to see the new clean, organized startup logs.
"""

import rospy
from middleware_logger import MiddlewareLogger

def test_publisher_logger():
    """Simulate a publisher node initialization."""
    logger = MiddlewareLogger("ROS → ZeroMQ Publisher", "/jackal1")
    
    # Simulate configuration setup
    logger.add_config("Robot ID", 1)
    logger.add_config("Other robots", "2: /jackal2, /jackal3")
    logger.add_config("Publisher endpoint", "tcp://192.168.0.101:5001")
    
    # Simulate connections
    logger.add_connection("tcp://192.168.0.101:5001", "Bound")
    
    # Simulate subscriptions
    logger.add_subscription("/jackal1/robot_to_robot/output/current_trajectory", "ObstacleGMM")
    logger.add_subscription("/jackal1/events/objective_reached", "Bool")
    
    # Display summary
    logger.log_startup_summary()


def test_subscriber_logger():
    """Simulate a subscriber node initialization."""
    logger = MiddlewareLogger("ZeroMQ → ROS Subscriber", "/jackal1")
    
    # Configuration
    logger.add_config("Robot ID", 1)
    logger.add_config("Listening to", "2 robots: /jackal2, /jackal3")
    
    # Publications
    logger.add_publication("/jackal2/robot_to_robot/output/current_trajectory", "ObstacleGMM")
    logger.add_publication("/jackal3/robot_to_robot/output/current_trajectory", "ObstacleGMM")
    logger.add_publication("/all_robots_reached_objective", "Bool")
    logger.add_publication("/jackal_deadman_switch", "Twist")
    
    # Subscriptions (ZeroMQ)
    logger.add_subscription("/jackal2/robot_to_robot/output/current_trajectory", "ZMQ")
    logger.add_subscription("/jackal3/robot_to_robot/output/current_trajectory", "ZMQ")
    logger.add_subscription("/all_robots_reached_objective", "ZMQ")
    logger.add_subscription("/jackal_deadman_switch", "ZMQ")
    
    # Connections
    logger.add_connection("tcp://192.168.0.102:5002", "Connected")
    logger.add_connection("tcp://192.168.0.103:5003", "Connected")
    logger.add_connection("tcp://192.168.0.100:4000", "Central Aggregator")
    
    # Display summary
    logger.log_startup_summary()


def test_central_aggregator_logger():
    """Simulate central aggregator initialization."""
    logger = MiddlewareLogger("Central Aggregator Publisher", "/central_aggregator")
    
    # Configuration
    logger.add_config("Robots", "3: /jackal1, /jackal2, /jackal3")
    logger.add_config("Publisher endpoint", "tcp://192.168.0.100:4000")
    
    # Connections
    logger.add_connection("tcp://192.168.0.100:4000", "Bound")
    
    # Subscriptions
    logger.add_subscription("/all_robots_reached_objective", "Bool")
    logger.add_subscription("/jackal_deadman_switch", "Twist")
    
    # Display summary
    logger.log_startup_summary()


if __name__ == "__main__":
    rospy.init_node('test_middleware_logging', log_level=rospy.INFO)
    
    print("\n\n========== TESTING PUBLISHER LOGGER ==========")
    test_publisher_logger()
    
    print("\n\n========== TESTING SUBSCRIBER LOGGER ==========")
    test_subscriber_logger()
    
    print("\n\n========== TESTING CENTRAL AGGREGATOR LOGGER ==========")
    test_central_aggregator_logger()
    
    print("\n\nTest complete! Compare this clean output to the old verbose logging.\n")
