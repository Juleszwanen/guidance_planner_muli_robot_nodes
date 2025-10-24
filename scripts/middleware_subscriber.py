#!/usr/bin/env python3
import rospy, zmq, json
from std_msgs.msg import String

ZMQ_CONNECT = "tcp://192.168.0.99:3001"
FILTER = b"/jackal1/trajectory_test_with_string"

# ROS must be initialized before creating pubs/subs
rospy.init_node("bridge_b_sub")

out_pub = rospy.Publisher("/remote/jackal1/trajectory_test_with_string", String, queue_size=5)
# out_pub.publish(String(data='hoi'))

ctx = zmq.Context.instance()
sub = ctx.socket(zmq.SUB)
sub.rcvhwm = 1000
sub.setsockopt(zmq.SUBSCRIBE, FILTER)
sub.connect(ZMQ_CONNECT)

rate = rospy.Rate(500)

while not rospy.is_shutdown():
    try:
        topic, meta_b, payload = sub.recv_multipart(flags=zmq.NOBLOCK)
        # meta = json.loads(meta_b.decode())  # if you need it
        msg = String()
        msg.deserialize(payload)
        print(msg)
        out_pub.publish(msg)
    except zmq.Again:
        rate.sleep()
