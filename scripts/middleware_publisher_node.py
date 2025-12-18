#!/usr/bin/env python3
import rospy
import sys
import os

# Add the scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

from middleware_ros_to_zeroMQ_class import MiddleWarePublisher

if __name__ == "__main__":
    try:
        rospy.init_node('middleware_publisher')
        middleware_pub = MiddleWarePublisher()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"MiddleWare Publisher node failed: {e}")
    finally:
        if 'middleware_pub' in locals():
            middleware_pub.cleanup()