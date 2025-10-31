#!/usr/bin/env python3
import rospy, zmq, json
import time
from genpy import Message
from std_msgs.msg import String
from io import StringIO, BytesIO

port = "3001"
TOPIC_OUT = "/jackal1/trajectory_test_with_string"   # absolute topic is safer
# ZMQ_BIND  = "tcp://*:3001"                           # bind on all ifaces, port 3001

# --- ZeroMQ setup ---
context = zmq.Context.instance()
socket = context.socket(zmq.PUB)
socket.sndhwm = 1000           # cap send queue; drop oldest if peer is slow
socket.setsockopt(zmq.LINGER, 0)  # don't hang on shutdown
socket.bind("tcp://*:{}".format(port))          # << you must bind before sending
time.sleep(1)

def to_bytes(msg: Message) -> bytes:
    # Works for any ROS message, including std_msgs/String
    buff = BytesIO()
    msg.serialize(buff)
    return buff.getvalue()

seq = 0
def cb(msg: String):
    global seq
    meta = {"type": "std_msgs/String", "seq": seq, "stamp": rospy.Time.now().to_sec(), "from": "jackal1" }
    seq += 1
    # When sending multi part everythin needs to be in bytes
    socket.send_multipart([
        TOPIC_OUT.encode(),                 # frame 0: topic
        json.dumps(meta).encode(),          # frame 1: metadata
        to_bytes(msg)                       # frame 2: ROS-serialized payload
    ])

if __name__ == "__main__":
    try:
        rospy.init_node("bridge_a_pub")
        rospy.Subscriber(TOPIC_OUT, String, cb, queue_size=5)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Bridge A publisher interrupted")
