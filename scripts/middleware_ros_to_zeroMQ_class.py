import rospy
import json
import zmq
from rospy import Subscriber, Publisher
from genpy import Message
from io import BytesIO
from util_middleware import extractRobotIdFromNamespace, identifyOtherRobotNamespaces
from mpc_planner_msgs.msg import ObstacleGMM
from std_msgs.msg import Bool
from rosbridge_library.internal import message_conversion
import threading





class MiddleWarePublisher:

    def __init__(self):
        # Instance variables - each object gets its own
        self.context = zmq.Context.instance()
        self.pub_socket = self.context.socket(zmq.PUB)
        
        self._ego_robot_ns = rospy.get_param("ego_robot_ns", "/jackalx")        # rospy.get_namespace().rstrip('/')
        rospy.logwarn(f"{self._ego_robot_ns}")
        self._ego_robot_id = extractRobotIdFromNamespace(self._ego_robot_ns)
        
        # Validate robot namespace format
        if self._ego_robot_id == -999:
            rospy.logerr(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] WRONG NAMESPACE FORMAT. Namespace should start with '/' and end with robot ID")
            return
        
        rospy.logdebug(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Extracted robot ID: {self._ego_robot_id}")
        
        # Get robot namespace list
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        rospy.logwarn(f"{self._robot_ns_list}")
        
        self._other_robots_nss = identifyOtherRobotNamespaces(self._robot_ns_list, self._ego_robot_ns)
        
        if len(self._other_robots_nss) == 0:
            rospy.logwarn(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Empty /robot_ns_list parameter. Check if parameter is set correctly")
        else:
            rospy.loginfo(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Found {len(self._other_robots_nss)} other robots: {self._other_robots_nss}")

        # Get network endpoint
        self._ego_robot_end_point = rospy.get_param(f"{self._ego_robot_ns}/network/publisher_end_point", "tcp://192.168.0.99:3999")
        
        if self._ego_robot_end_point == "tcp://192.168.0.99:3999":
            rospy.logwarn(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Using default endpoint for itself{self._ego_robot_end_point}. Set publisher_end_point in configuration file")
        
        # Bind socket
        self.bind_pub_socket()

        # Setup ROS subscriber, the node subscribes to its own output trajectory
        self._ego_robot_trajectory_topic = f"{self._ego_robot_ns.rstrip('/')}/robot_to_robot/output/current_trajectory"
        # Each robot publishes its individual objective reached status to its own namespace
        self._ego_robot_objective_reached_topic = f"{self._ego_robot_ns.rstrip('/')}/events/objective_reached"

        self._ego_trajectory_subscriber = rospy.Subscriber(
            self._ego_robot_trajectory_topic, 
            ObstacleGMM, 
            self.trajectory_callback
        )

        self._ego_objective_reached_subscriber = rospy.Subscriber(
            self._ego_robot_objective_reached_topic, 
            Bool, 
            self.objective_reached_callback
        )

        self.seq = 0
        
        rospy.loginfo(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Middleware initialized successfully - Ready to relay ROS messages to ZeroMQ network")
        
    def to_bytes(self, msg):
        """Convert ROS message to bytes for ZeroMQ transmission."""
        try:
            buff = BytesIO()
            msg.serialize(buff)
            return buff.getvalue()
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Failed to serialize message to binary: {e}")
            return b""
    
    def to_json(self, msg):
        """Convert ROS message to JSON string (placeholder for future implementation)."""
        # TODO: Implement JSON serialization if needed
        try:
            py_obj = message_conversion(msg)
            return json.dumps(py_obj)
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Failed to serialize message to json: {e}")
            return "{}"

    def bind_pub_socket(self):
        """Bind ZeroMQ publisher socket to endpoint."""
        try:
            self.pub_socket.bind(self._ego_robot_end_point)
            rospy.loginfo(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Successfully bound to {self._ego_robot_end_point}")
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Failed to bind to {self._ego_robot_end_point}: {e}")
        
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
                self._ego_robot_trajectory_topic.encode(),
                json.dumps(meta_data).encode(),
                self.to_bytes(msg)
            ])
            
            rospy.logdebug(f"{self._ego_robot_ns}: [ROS → ZeroMQ] Relayed trajectory message to network seq={self.seq}")
            
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Failed to relay trajectory message to ZeroMQ: {e}")


    def objective_reached_callback(self, msg):
        try:
            # Increment sequence number (fixed: use self.seq)
            self.seq += 1
    
            # Create metadata
            meta_data = {
                "type": "Bool", 
                "seq": self.seq,
                "stamp": rospy.Time.now().to_sec(), 
                "from": self._ego_robot_ns
            }
            
            # Send multipart message
            self.pub_socket.send_multipart([
                self._ego_robot_objective_reached_topic.encode(),
                json.dumps(meta_data).encode(),
                self.to_bytes(msg)
            ])
           
            
            rospy.loginfo(f"{self._ego_robot_ns}: [ROS → ZeroMQ] Relayed objective_reached (status={msg.data}) to network")
            
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Failed to relay objective_reached message to ZeroMQ: {e}")


    def cleanup(self):
        """Cleanup resources."""
        try:
            self.pub_socket.close()
            rospy.loginfo(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Socket closed")
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ROS → ZeroMQ Publisher] Error during cleanup: {e}")

    def get_namespace(self):
        return self._ego_robot_ns
    
    def get_robot_id(self):
        return self._ego_robot_id


class MiddleWareSubscriber:
    def __init__(self):
        # Instance variables - each object gets its own
        self.context = zmq.Context.instance()

        # A subscriber can connect to many publishers endpoints, Now down only via 1 socket, can also be done be creating a socket for every endpoint
        self.sub_socket = self.context.socket(zmq.SUB)
        
        self._ego_robot_ns = rospy.get_param("ego_robot_ns", "/jackalx") # rospy.get_namespace().rstrip('/')
        self._ego_robot_id = extractRobotIdFromNamespace(self._ego_robot_ns)
        
        # Validate robot namespace format
        if self._ego_robot_id == -999:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] WRONG NAMESPACE FORMAT. Namespace should start with '/' and end with robot ID")
            return
        
        rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Extracted robot ID: {self._ego_robot_id}")
        
        # Get robot namespace list
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        self._other_robots_nss = identifyOtherRobotNamespaces(self._robot_ns_list, self._ego_robot_ns)
        
        if len(self._other_robots_nss) == 0:
            rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Empty /robot_ns_list parameter. Check if parameter is set correctly")
        else:
            rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Found {len(self._other_robots_nss)} other robots to listen to: {self._other_robots_nss}")

        # Create ROS publishers for each other robot
        self._other_robot_ros_publisher_dict = self.create_other_robot_trajectory_ros_publishers()
        self._central_aggregator_ros_publisher = rospy.Publisher("/all_robots_reached_objective", Bool, queue_size=10)

        # Setup ZeroMQ subscriber endpoints
        self._other_robot_subscriber_endpoints = self.get_other_robot_subscriber_endpoints()
        self._central_aggregator_endpoint      = self.get_central_aggregator_endpoint()
        self.setup_zmq_subscriber()
        
        # Start message processing thread
        self._running = True
        
        self._message_thread = threading.Thread(target=self.message_processing_loop)
        self._message_thread.daemon = True
        self._message_thread.start()
        
        rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Middleware initialized successfully - Ready to receive ZeroMQ messages and relay to ROS")

    def create_other_robot_trajectory_ros_publishers(self):
        """Create ROS publishers for each other robot's trajectory topic."""
        publisher_dict_holder = {}
    
        for other_ns in self._other_robots_nss:
            # Double check (though this should not happen after identifyOtherRobotNamespaces)
            if other_ns == self._ego_robot_ns:
                rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Skipping own namespace {other_ns} in publisher creation")
                continue
            
            try:
                # Create topic name for the other robot's trajectory to which the planner node is normally subscribed
                topic_name = f"{other_ns.rstrip('/')}/robot_to_robot/output/current_trajectory"
                publisher = rospy.Publisher(topic_name, ObstacleGMM, queue_size=10)
                publisher_dict_holder[other_ns] = publisher
                
                rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Created ROS publisher for relaying {other_ns} trajectory on topic {topic_name}")
                
            except Exception as e:
                rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to create ROS publisher for {other_ns}: {e}")
        
        rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Created {len(publisher_dict_holder)} ROS publishers for relaying")
        return publisher_dict_holder
       
    def get_other_robot_subscriber_endpoints(self):
        """Get ZeroMQ subscriber endpoints for other robots."""
        endpoints = []
        
        for other_ns in self._other_robots_nss:
            try:
                # Get the publisher endpoint for this robot
                endpoint = rospy.get_param(f"{other_ns}/network/publisher_end_point", None)
                
                if endpoint:
                    endpoints.append(endpoint)
                    rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Will connect to ZeroMQ endpoint of {other_ns} at {endpoint}")
                else:
                    rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] No publisher_end_point found for {other_ns}")
                    
            except Exception as e:
                rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to get endpoint for {other_ns}: {e}")
        
        return endpoints

    def get_central_aggregator_endpoint(self):
        endpoint = None
        try:
            # Get the publisher endpoint for this robot
            endpoint = rospy.get_param(f"/central_aggregator/network/publisher_end_point", None)
            
            if endpoint:
                rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Will connect to ZeroMQ central aggregator at {endpoint}")
    
            else:
                rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] No central aggregator publisher_end_point found")
                    
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to get endpoint for central aggregator: {e}")
        return endpoint
        
    def setup_zmq_subscriber(self):
        """Setup ZeroMQ subscriber socket and connect to other robots."""
        try:
            # Subscribe to specific topics instead of all messages for better performance
            for other_ns in self._other_robots_nss:
                # Subscribe to trajectory topics from other robots
                trajectory_topic = f"{other_ns.rstrip('/')}/robot_to_robot/output/current_trajectory"
                self.sub_socket.setsockopt(zmq.SUBSCRIBE, trajectory_topic.encode())
                rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Subscribed to ZeroMQ topic: {trajectory_topic}")
            
            # Subscribe to the global aggregated objective reached topic from central aggregator
            global_objective_topic = "/all_robots_reached_objective"
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, global_objective_topic.encode())
            rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Subscribed to ZeroMQ global objective topic: {global_objective_topic}")

            
            # Connect to all other robot endpoints
            for endpoint in self._other_robot_subscriber_endpoints:
                try:
                    self.sub_socket.connect(endpoint)
                    rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Connected to ZeroMQ endpoint: {endpoint}")
                except Exception as e:
                    rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to connect to {endpoint}: {e}")

            # Connect to central aggregator if endpoint exists
            if self._central_aggregator_endpoint:
                try:
                    self.sub_socket.connect(self._central_aggregator_endpoint)
                    rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Connected to central aggregator ZeroMQ endpoint: {self._central_aggregator_endpoint}")
                except Exception as e:
                    rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to connect to central aggregator: {e}")
            else:
                rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] No central aggregator endpoint to connect to")
                    
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to setup subscriber: {e}")

    def from_bytes(self, data, msg_class):
        """Convert bytes back to ROS message of specified type."""
        try:
            msg = msg_class()
            buff = BytesIO(data)
            msg.deserialize(buff.getvalue())
            return msg
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ Subscriber] Failed to deserialize {msg_class.__name__}: {e}")
            return msg_class()  # Return empty message on error

    def _handle_trajectory_message(self, sender_ns, msg_bytes, metadata):
        """Handle trajectory messages."""
        try:
            trajectory_msg = self.from_bytes(msg_bytes, ObstacleGMM)
            
            if sender_ns in self._other_robot_ros_publisher_dict:
                self._other_robot_ros_publisher_dict[sender_ns].publish(trajectory_msg)
                rospy.logdebug(f"{self._ego_robot_ns}: [ZeroMQ → ROS] Relayed trajectory from {sender_ns} to local ROS topic seq={metadata.get('seq', 'unknown')}")
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to handle trajectory message: {e}")

    def _handle_objective_reached_message(self, sender_ns, msg_bytes, metadata, topic):
        """Handle objective reached messages."""
        try:
            objective_msg = self.from_bytes(msg_bytes, Bool)
            
            # Determine if this is an individual robot message or global aggregator message
            if topic == "/all_robots_reached_objective":
                # This is from the central aggregator - publish to our local aggregated topic
                self._central_aggregator_ros_publisher.publish(objective_msg)
                rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS] Relayed GLOBAL objective_reached from central aggregator to local ROS, status: {objective_msg.data}")
            else:
                # This is from an individual robot - could relay to a per-robot topic if needed
                rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS] Received individual objective_reached from {sender_ns}, status: {objective_msg.data} (not relaying)")
                # Note: Currently not relaying individual robot objective messages to ROS topics
                # Could add individual robot objective publishers here if needed
            
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to handle objective reached message: {e}")

    def message_processing_loop(self):
        """Main loop for processing incoming ZeroMQ messages."""
        rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Started message processing loop")
        
        while self._running and not rospy.is_shutdown():
            try:
                if self.sub_socket.poll(timeout=100):  # 100ms timeout
                    multipart_msg = self.sub_socket.recv_multipart(zmq.NOBLOCK)
                    
                    if len(multipart_msg) == 3:
                        topic_bytes, metadata_bytes, msg_bytes = multipart_msg
                        
                        # Decode topic and metadata
                        topic = topic_bytes.decode()
                        metadata = json.loads(metadata_bytes.decode())
                        sender_ns = metadata.get("from", "unknown")
                        msg_type = metadata.get("type", "unknown")
                        
                        # Skip our own messages
                        if sender_ns == self._ego_robot_ns:
                            continue
                        
                        # Handle different message types
                        if msg_type == "ObstacleGMM":
                            self._handle_trajectory_message(sender_ns, msg_bytes, metadata)
                        elif msg_type == "Bool":
                            self._handle_objective_reached_message(sender_ns, msg_bytes, metadata, topic)
                        else:
                            rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Unknown message type received from ZeroMQ: {msg_type}")
                            
            except zmq.Again:
                continue
            except Exception as e:
                rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Error in message processing loop: {e}")

    def cleanup(self):
        """Cleanup resources."""
        try:
            self._running = False
            
            # Wait for thread to finish
            if hasattr(self, '_message_thread') and self._message_thread.is_alive():
                self._message_thread.join(timeout=1.0)
            
            # Close socket
            self.sub_socket.close()
            rospy.loginfo(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Socket closed")
            
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Error during cleanup: {e}")

    def get_namespace(self):
        return self._ego_robot_ns
    
    def get_robot_id(self):
        return self._ego_robot_id
    
class MiddleWareCentralAggBase:
    def __init__(self):
        self.context    = zmq.Context.instance()
        self.sub_socket = None
        self.pub_socket = None
        self._ego_node_end_point= None
        self._robot_ns_list = []

        self._other_robot_subscriber_endpoints = []
        self._ego_node_all_robots_reached_objective_topic = None
        self._robots_objective_reached_publisher_dict = {}


class MiddleWareCentralAggBasePub(MiddleWareCentralAggBase):
    def __init__(self):
        super().__init__()
        
        # Initialize ZeroMQ publisher socket
        self.pub_socket = self.context.socket(zmq.PUB)
        
        # Get central aggregator endpoint configuration
        self._ego_node_end_point = rospy.get_param("/central_aggregator/network/publisher_end_point", "tcp://192.168.0.99:4000")
        
        # The topic this central aggregator publishes to
        self._ego_node_all_robots_reached_objective_topic = "/all_robots_reached_objective"
        
        # Get list of robots to aggregate
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        
        # Bind publisher socket
        self.bind_pub_socket()
        
        # ROS subscriber to listen for local aggregation trigger
        # (This would be where your aggregation logic publishes locally)
        self._local_aggregator_subscriber = rospy.Subscriber(
            self._ego_node_all_robots_reached_objective_topic,
            Bool,
            self.aggregate_callback
        )
        
        self.seq = 0
        
    def bind_pub_socket(self):
        """Bind ZeroMQ publisher socket."""
        try:
            self.pub_socket.bind(self._ego_node_end_point)
            rospy.loginfo(f"[Central Aggregator ROS → ZeroMQ Publisher] Bound to {self._ego_node_end_point}")
        except Exception as e:
            rospy.logerr(f"[Central Aggregator ROS → ZeroMQ Publisher] Failed to bind: {e}")
    
    def to_bytes(self, msg):
        """Convert ROS message to bytes for ZeroMQ transmission."""
        try:
            buff = BytesIO()
            msg.serialize(buff)
            return buff.getvalue()
        except Exception as e:
            rospy.logerr(f"[Central Aggregator ROS → ZeroMQ Publisher] Failed to serialize message to binary: {e}")
            return b""

    def aggregate_callback(self, msg):
        """Callback to publish aggregated objective status."""
        try:
            self.seq += 1
            
            meta_data = {
                "type": "Bool",
                "seq": self.seq,
                "stamp": rospy.Time.now().to_sec(),
                "from": "/central_aggregator"
            }
            
            # Publish via ZeroMQ
            self.pub_socket.send_multipart([
                self._ego_node_all_robots_reached_objective_topic.encode(),
                json.dumps(meta_data).encode(),
                self.to_bytes(msg)
            ])
            
            rospy.loginfo(f"[Central Aggregator ROS → ZeroMQ] Relayed global objective_reached (status={msg.data}) to ZeroMQ network")
            
        except Exception as e:
            rospy.logerr(f"[Central Aggregator ROS → ZeroMQ Publisher] Failed to relay to ZeroMQ: {e}")
    
    def cleanup(self):
        """Cleanup resources."""
        try:
            self.pub_socket.close()
            rospy.loginfo(f"[Central Aggregator ROS → ZeroMQ Publisher] Socket closed")
        except Exception as e:
            rospy.logerr(f"[Central Aggregator ROS → ZeroMQ Publisher] Error during cleanup: {e}")


class MiddleWareCentralAggBaseSub(MiddleWareCentralAggBase):
    def __init__(self):
        super().__init__()
        
        # Initialize ZeroMQ subscriber socket
        self.sub_socket = self.context.socket(zmq.SUB)
        
        # Get robot namespace list
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        self._other_robot_subscriber_endpoints = self.get_robot_endpoints()
        
        # Create ROS publishers for each robot's objective reached topic
        self._robots_objective_reached_publisher_dict = self.create_robot_objective_publishers()
        
        # Setup ZeroMQ subscriptions
        self.setup_zmq_subscriber()
        
        # Start message processing thread
        self._running = True
        self._message_thread = threading.Thread(target=self.message_processing_loop)
        self._message_thread.daemon = True
        self._message_thread.start()
    
    def get_robot_endpoints(self):
        """Get endpoints for all robots."""
        endpoints = []
        for robot_ns in self._robot_ns_list:
            try:
                endpoint = rospy.get_param(f"{robot_ns}/network/publisher_end_point", None)
                if endpoint:
                    endpoints.append(endpoint)
            except Exception as e:
                rospy.logerr(f"[Central Aggregator ZeroMQ → ROS Subscriber] Failed to get endpoint for {robot_ns}: {e}")
        return endpoints
    
    def create_robot_objective_publishers(self):
        """Create ROS publishers for each robot's objective reached topic."""
        publisher_dict = {}
        for robot_ns in self._robot_ns_list:
            try:
                # Create local ROS topic for this robot's objective status
                topic_name = f"{robot_ns.rstrip('/')}/events/objective_reached"
                publisher = rospy.Publisher(topic_name, Bool, queue_size=10)
                publisher_dict[robot_ns] = publisher
                
                rospy.loginfo(f"[Central Aggregator ZeroMQ → ROS Subscriber] Created ROS publisher for relaying {robot_ns} on topic {topic_name}")
            except Exception as e:
                rospy.logerr(f"[Central Aggregator ZeroMQ → ROS Subscriber] Failed to create ROS publisher for {robot_ns}: {e}")
        return publisher_dict
    
    def setup_zmq_subscriber(self):
        """Setup ZeroMQ subscriber."""
        try:
            # Subscribe to each robot's objective reached topic
            for robot_ns in self._robot_ns_list:
                objective_topic = f"{robot_ns.rstrip('/')}/events/objective_reached"
                self.sub_socket.setsockopt(zmq.SUBSCRIBE, objective_topic.encode())
                rospy.loginfo(f"[Central Aggregator ZeroMQ → ROS Subscriber] Subscribed to ZeroMQ topic: {objective_topic}")
            
            # Connect to all robot endpoints
            for endpoint in self._other_robot_subscriber_endpoints:
                self.sub_socket.connect(endpoint)
                rospy.loginfo(f"[Central Aggregator ZeroMQ → ROS Subscriber] Connected to ZeroMQ endpoint: {endpoint}")
                
        except Exception as e:
            rospy.logerr(f"[Central Aggregator ZeroMQ → ROS Subscriber] Setup failed: {e}")
    
    def message_processing_loop(self):
        """Process incoming robot objective messages and relay to ROS."""
        while self._running and not rospy.is_shutdown():
            try:
                if self.sub_socket.poll(timeout=100):
                    multipart_msg = self.sub_socket.recv_multipart(zmq.NOBLOCK)
                    
                    if len(multipart_msg) == 3:
                        topic_bytes, metadata_bytes, msg_bytes = multipart_msg
                        
                        topic = topic_bytes.decode()
                        metadata = json.loads(metadata_bytes.decode())
                        sender_ns = metadata.get("from", "unknown")
                        
                        # Deserialize the Bool message
                        objective_msg = self.from_bytes(msg_bytes, Bool)
                        
                        # Simply republish to local ROS topic
                        if sender_ns in self._robots_objective_reached_publisher_dict:
                            self._robots_objective_reached_publisher_dict[sender_ns].publish(objective_msg)
                            rospy.loginfo(f"[Central Aggregator ZeroMQ → ROS] Relayed objective_reached from {sender_ns} to local ROS topic, status: {objective_msg.data}")
                        
            except zmq.Again:
                continue
            except Exception as e:
                rospy.logerr(f"[Central Aggregator ZeroMQ → ROS Subscriber] Processing error: {e}")

    def from_bytes(self, data, msg_class):
        """Convert bytes back to ROS message of specified type."""
        try:
            msg = msg_class()
            buff = BytesIO(data)
            msg.deserialize(buff.getvalue())
            return msg
        except Exception as e:
            rospy.logerr(f"[Central Aggregator ZeroMQ → ROS Subscriber] Failed to deserialize {msg_class.__name__}: {e}")
            return msg_class()  # Return empty message on error

    def cleanup(self):
        """Cleanup resources."""
        try:
            self._running = False
            
            # Wait for thread to finish
            if hasattr(self, '_message_thread') and self._message_thread.is_alive():
                self._message_thread.join(timeout=1.0)
            
            # Close socket
            self.sub_socket.close()
            rospy.loginfo(f"[Central Aggregator ZeroMQ → ROS Subscriber] Socket closed")
            
        except Exception as e:
            rospy.logerr(f"[Central Aggregator ZeroMQ → ROS Subscriber] Error during cleanup: {e}")