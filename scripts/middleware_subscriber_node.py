#!/usr/bin/env python3
# filepath: /workspace/src/guidance_planner_multi_robot_nodes/scripts/middleware_subscriber_node.py
import rospy
import sys
import os

# Add the scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

from middleware_ros_to_zeroMQ_class import MiddleWareSubscriber

if __name__ == "__main__":
    try:
        rospy.init_node('middleware_subscriber')
        middleware_sub = MiddleWareSubscriber()
        
        rospy.loginfo(f"{middleware_sub._ego_robot_ns}: MiddleWare Subscriber node started")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("MiddleWare Subscriber node interrupted")
    except Exception as e:
        rospy.logerr(f"MiddleWare Subscriber node failed: {e}")
    finally:
        if 'middleware_sub' in locals():
            middleware_sub.cleanup()