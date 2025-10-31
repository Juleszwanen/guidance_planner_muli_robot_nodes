import rospy
import json
import zmq
from rospy import Subscriber, Publisher
from genpy import Message
from io import BytesIO
from util_middleware import extractRobotIdFromNamespace, identifyOtherRobotNamespaces
from mpc_planner_msgs.msg import ObstacleGMM
from rosbridge_library.internal import message_conversion
import threading

class MiddleWarePublisher:

    def __init__(self):
        # Instance variables - each object gets its own
        self.context = zmq.Context.instance()
        self.pub_socket = self.context.socket(zmq.PUB)
        
        self._ego_robot_ns = rospy.get_namespace().rstrip('/')
        rospy.logwarn(f"{self._ego_robot_ns}")
        self._ego_robot_id = extractRobotIdFromNamespace(self._ego_robot_ns)
        
        # Validate robot namespace format
        if self._ego_robot_id == -999:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Publisher] WRONG NAMESPACE FORMAT. Namespace should start with '/' and end with robot ID")
            return
        
        rospy.loginfo(f"{self._ego_robot_ns}: Extracted robot ID: {self._ego_robot_id}")
        
        # Get robot namespace list
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        rospy.logwarn(f"{self._robot_ns_list}")
        
        self._other_robots_nss = identifyOtherRobotNamespaces(self._robot_ns_list, self._ego_robot_ns)
        
        if len(self._other_robots_nss) == 0:
            rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Empty /robot_ns_list parameter. Check if parameter is set correctly")
        else:
            rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Found {len(self._other_robots_nss)} other robots: {self._other_robots_nss}")

        # Get network endpoint
        self._ego_robot_end_point = rospy.get_param("network/publisher_end_point", "tcp://192.168.0.99:3999")
        
        if self._ego_robot_end_point == "tcp://192.168.0.99:3999":
            rospy.logwarn(f"{self._ego_robot_ns}: Using default endpoint {self._ego_robot_end_point}. Set publisher_end_point in configuration file")
        
        # Bind socket
        self.bind_pub_socket()

        # Setup ROS subscriber, the node subcribes to its own output trajectory b
        self._ego_robot_trajectory_topic = "robot_to_robot/output/current_trajectory"
        self._absolute_ego_robot_trajectory_topic = f"{self._ego_robot_ns.rstrip('/')}/robot_to_robot/output/current_trajectory"
        self._ego_trajectory_subscriber = rospy.Subscriber(
            self._ego_robot_trajectory_topic, 
            ObstacleGMM, 
            self.trajectory_callback
        )

        self.seq = 0
        
        rospy.loginfo(f"{self._ego_robot_ns}: MiddleWarePublisher initialized successfully")
        
    def to_bytes(self, msg):
        """Convert ROS message to bytes for ZeroMQ transmission."""
        try:
            buff = BytesIO()
            msg.serialize(buff)
            return buff.getvalue()
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Failed to serialize message to binary: {e}")
            return b""
    
    def to_json(self, msg):
        """Convert ROS message to JSON string (placeholder for future implementation)."""
        # TODO: Implement JSON serialization if needed
        try:
            py_obj = message_conversion(msg)
            return json.dumps(py_obj)
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Failed to serialize message to json: {e}")
            return "{}"

    def bind_pub_socket(self):
        """Bind ZeroMQ publisher socket to endpoint."""
        try:
            self.pub_socket.bind(self._ego_robot_end_point)
            rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Successfully bound to {self._ego_robot_end_point}")
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Failed to bind to {self._ego_robot_end_point}: {e}")
        
    def trajectory_callback(self, msg):
        """Callback for trajectory messages - forwards to ZeroMQ."""
        try:
            # Increment sequence number (fixed: use self.seq)
            self.seq += 1
            
            # Create metadata
            meta_data = {
                "type": "ObstacleGMM", 
                "seq": self.seq, 
                "stamp": rospy.Time.now().to_sec(), 
                "from": self._ego_robot_ns
            }
            
            # Send multipart message
            self.pub_socket.send_multipart([
                self._absolute_ego_robot_trajectory_topic.encode(),
                json.dumps(meta_data).encode(),
                self.to_bytes(msg)
            ])
            
            rospy.logdebug(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Published trajectory message seq={self.seq}")
            
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Failed to publish trajectory message: {e}")

    def cleanup(self):
        """Cleanup resources."""
        try:
            self.pub_socket.close()
            rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Socket closed")
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Publisher] Error during cleanup: {e}")

    def get_namespace(self):
        return self._ego_robot_ns
    
    def get_robot_id(self):
        return self._ego_robot_ns




class MiddleWareSubscriber:
    def __init__(self):
        # Instance variables - each object gets its own
        self.context = zmq.Context.instance()

        # A subscriber can connect to many publishers, Now down only via 1 socket, can also be done be creating a socket for every endpoint
        self.sub_socket = self.context.socket(zmq.SUB)
        
        self._ego_robot_ns = rospy.get_namespace().rstrip('/')
        self._ego_robot_id = extractRobotIdFromNamespace(self._ego_robot_ns)
        
        # Validate robot namespace format
        if self._ego_robot_id == -999:
            rospy.logerr(f"{self._ego_robot_ns}: WRONG NAMESPACE FORMAT. Namespace should start with '/' and end with robot ID")
            return
        
        rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Extracted robot ID: {self._ego_robot_id}")
        
        # Get robot namespace list
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        self._other_robots_nss = identifyOtherRobotNamespaces(self._robot_ns_list, self._ego_robot_ns)
        
        if len(self._other_robots_nss) == 0:
            rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Empty /robot_ns_list parameter. Check if parameter is set correctly")
        else:
            rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber]Found {len(self._other_robots_nss)} other robots: {self._other_robots_nss}")

        # Create ROS publishers for each other robot
        self.publisher_dict = self.create_publishers()
        
        # Setup ZeroMQ subscriber endpoints
        self._subscriber_endpoints = self.get_subscriber_endpoints()
        self.setup_zmq_subscriber()
        
        # Start message processing thread
        self._running = True
        
        self._message_thread = threading.Thread(target=self.message_processing_loop)
        self._message_thread.daemon = True
        self._message_thread.start()
        
        rospy.loginfo(f"{self._ego_robot_ns}: MiddleWareSubscriber initialized successfully")

    def create_publishers(self):
        """Create ROS publishers for each other robot's trajectory topic."""
        publisher_dict_holder = {}
        
        for other_ns in self._other_robots_nss:
            # Double check (though this should not happen after identifyOtherRobotNamespaces)
            if other_ns == self._ego_robot_ns:
                rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Skipping own namespace {other_ns} in publisher creation")
                continue
            
            try:
                # Create topic name for the other robot's trajectory to which the planner node is normally subscribed
                topic_name = f"{other_ns.rstrip('/')}/robot_to_robot/output/current_trajectory"
                publisher = rospy.Publisher(topic_name, ObstacleGMM, queue_size=10)
                publisher_dict_holder[other_ns] = publisher
                
                rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Created publisher for {other_ns} on topic {topic_name}")
                
            except Exception as e:
                rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Failed to create publisher for {other_ns}: {e}")
        
        rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Created {len(publisher_dict_holder)} ROS publishers")
        return publisher_dict_holder

    def get_subscriber_endpoints(self):
        """Get ZeroMQ subscriber endpoints for other robots."""
        endpoints = []
        
        for other_ns in self._other_robots_nss:
            try:
                # Get the publisher endpoint for this robot
                endpoint = rospy.get_param(f"{other_ns}/network/publisher_end_point", None)
                
                if endpoint:
                    endpoints.append(endpoint)
                    rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Will subscribe to {other_ns} at {endpoint}")
                else:
                    rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] No publisher_end_point found for {other_ns}")
                    
            except Exception as e:
                rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Failed to get endpoint for {other_ns}: {e}")
        
        return endpoints

    def setup_zmq_subscriber(self):
        """Setup ZeroMQ subscriber socket and connect to other robots."""
        try:
            # Subscribe to all messages (empty filter)
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, b"")
            
            # Connect to all other robot endpoints
            for endpoint in self._subscriber_endpoints:
                try:
                    self.sub_socket.connect(endpoint)
                    rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Connected to endpoint: {endpoint}")
                except Exception as e:
                    rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Failed to connect to {endpoint}: {e}")
                    
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Failed to setup subscriber: {e}")

    def from_bytes(self, data):
        """Convert bytes back to ROS message."""
        try:
            msg = ObstacleGMM()
            buff = BytesIO(data)
            msg.deserialize(buff.getvalue())
            return msg
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Failed to deserialize message: {e}")
            return ObstacleGMM()  # Return empty message on error

    def message_processing_loop(self):
        """Main loop for processing incoming ZeroMQ messages."""
        rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Started message processing loop")
        
        while self._running and not rospy.is_shutdown():
            try:
                # Non-blocking receive with timeout
                if self.sub_socket.poll(timeout=100):  # 100ms timeout
                    # Receive multipart message: [topic, metadata, message_data]
                    multipart_msg = self.sub_socket.recv_multipart(zmq.NOBLOCK)
                    
                    if len(multipart_msg) == 3:
                        topic_bytes, metadata_bytes, msg_bytes = multipart_msg
                        
                        # Decode metadata
                        metadata = json.loads(metadata_bytes.decode())
                        sender_ns = metadata.get("from", "unknown")
                        
                        # Skip our own messages (shouldn't happen, but safety check)
                        if sender_ns == self._ego_robot_ns:
                            continue
                        
                        # Deserialize and republish message
                        trajectory_msg = self.from_bytes(msg_bytes)
                        
                        if sender_ns in self.publisher_dict:
                            self.publisher_dict[sender_ns].publish(trajectory_msg)
                            rospy.logdebug(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Relayed message from {sender_ns} seq={metadata.get('seq', 'unknown')}")
                        else:
                            rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] No publisher found for sender {sender_ns}")
                    
                    else:
                        rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Received malformed multipart message with {len(multipart_msg)} parts")
                        
            except zmq.Again:
                # No message available, continue
                continue
            except Exception as e:
                rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Error in message processing loop: {e}")
                
        rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Message processing loop stopped")

    def cleanup(self):
        """Cleanup resources."""
        try:
            self._running = False
            
            # Wait for thread to finish
            if hasattr(self, '_message_thread') and self._message_thread.is_alive():
                self._message_thread.join(timeout=1.0)
            
            # Close socket
            self.sub_socket.close()
            rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Socket closed")
            
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Error during cleanup: {e}")

    def get_namespace(self):
        return self._ego_robot_ns
    
    def get_robot_id(self):
        return self._ego_robot_ns