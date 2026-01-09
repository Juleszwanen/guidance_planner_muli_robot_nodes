import rospy
import json
import zmq
from rospy import Subscriber, Publisher
from genpy import Message
from io import BytesIO
from util_middleware import extractRobotIdFromNamespace, identifyOtherRobotNamespaces
from mpc_planner_msgs.msg import ObstacleGMM
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from rosbridge_library.internal import message_conversion
from middleware_logger import MiddlewareLogger
import threading
import time




class MiddleWarePublisher:

    def __init__(self):
        # Instance variables - each object gets its own
        self.context = zmq.Context.instance()
        self.pub_socket = self.context.socket(zmq.PUB)
        self._socket_lock = threading.Lock()  # Prevent interleaved sends
        
        self._ego_robot_ns = rospy.get_param("ego_robot_ns", "/jackalx")
        self._ego_robot_id = extractRobotIdFromNamespace(self._ego_robot_ns)
        
        # Initialize centralized logger
        self.logger = MiddlewareLogger("ROS → ZeroMQ Publisher", self._ego_robot_ns)
        
        # Validate robot namespace format
        if self._ego_robot_id == -999:
            self.logger.add_error("Invalid namespace format. Must start with '/' and end with robot ID")
            self.logger.log_startup_summary()
            return
        
        self.logger.add_config("Robot ID", self._ego_robot_id)
        
        # Get robot namespace list
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        self._other_robots_nss = identifyOtherRobotNamespaces(self._robot_ns_list, self._ego_robot_ns)
        
        if len(self._other_robots_nss) == 0:
            self.logger.add_warning("Empty /robot_ns_list parameter")
        else:
            self.logger.add_config("Other robots", f"{len(self._other_robots_nss)}: {', '.join(self._other_robots_nss)}")

        # Get network endpoint
        self._ego_robot_end_point = rospy.get_param(f"{self._ego_robot_ns}/network/publisher_end_point", "tcp://192.168.0.99:3999")
        
        if self._ego_robot_end_point == "tcp://192.168.0.99:3999":
            self.logger.add_warning(f"Using default endpoint {self._ego_robot_end_point}")
        
        self.logger.add_config("Publisher endpoint", self._ego_robot_end_point)
        
        # Bind socketund I made was to create a client that manually drops old queued messages. Googling I found many people have asked for the same t
        self.bind_pub_socket()

        # Setup ROS subscriber, the node subscribes to its own output trajectory
        self._ego_robot_trajectory_topic = f"{self._ego_robot_ns.rstrip('/')}/robot_to_robot/output/current_trajectory"
        # self._ego_robot_objective_reached_topic = f"{self._ego_robot_ns.rstrip('/')}/events/objective_reached"
        self._ego_robot_objective_reached_topic = f"{self._ego_robot_ns.rstrip('/')}/events/objective_reached"
        self._ego_robot_objective_reached_zeromq_topic = f"{self._ego_robot_id}{self._ego_robot_id}{self._ego_robot_id}{self._ego_robot_ns.rstrip('/')}/events/objective_reached"
        
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
        
        self.logger.add_subscription(self._ego_robot_trajectory_topic, "ObstacleGMM")
        self.logger.add_subscription(self._ego_robot_objective_reached_topic, "Bool")

        self.seq = 0
        
        # Log the complete startup summary once
        self.logger.log_startup_summary()
        
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
            self.logger.add_connection(self._ego_robot_end_point, "Bound")
        except Exception as e:
            self.logger.add_error(f"Failed to bind to {self._ego_robot_end_point}: {e}")
        
    def trajectory_callback(self, msg):
        """Callback for trajectory messages - forwards to ZeroMQ."""
        try:
            self.seq += 1
            
            meta_data = {
                "type": "ObstacleGMM", 
                "seq": self.seq, 
                "stamp": rospy.Time.now().to_sec(), 
                "from": self._ego_robot_ns
            }
            
            with self._socket_lock:  # Atomic multipart send
                self.pub_socket.send_multipart([
                    self._ego_robot_trajectory_topic.encode(),
                    json.dumps(meta_data).encode(),
                    self.to_bytes(msg)
                ])
            
            rospy.logdebug(f"{self._ego_robot_ns}: [ROS → ZeroMQ] Relayed trajectory seq={self.seq}")
            
        except Exception as e:
            self.logger.log_runtime_event("ROS → ZeroMQ", f"Failed to relay trajectory: {e}", "error")

    def objective_reached_callback(self, msg):
        try:
            self.seq += 1
    
            meta_data = {
                "type": "Bool", 
                "seq": self.seq,
                "stamp": rospy.Time.now().to_sec(), 
                "from": self._ego_robot_ns
            }
            
            # self.pub_socket.send_multipart([
            #     self._ego_robot_objective_reached_topic.encode(),
            #     json.dumps(meta_data).encode(),
            #     self.to_bytes(msg)
            # ])

            with self._socket_lock:  # Atomic multipart send
                self.pub_socket.send_multipart([
                    self._ego_robot_objective_reached_zeromq_topic.encode(),
                    json.dumps(meta_data).encode(),
                    self.to_bytes(msg)
                ])
            
            self.logger.log_runtime_event("ROS → ZeroMQ", f"Relayed objective_reached={msg.data}", "info")
            
        except Exception as e:
            self.logger.log_runtime_event("ROS → ZeroMQ", f"Failed to relay objective_reached: {e}", "error")

    def cleanup(self):
        """Cleanup resources."""
        try:
            self.pub_socket.close()
            self.logger.log_runtime_event("Shutdown", "Socket closed", "info")
        except Exception as e:
            self.logger.log_runtime_event("Shutdown", f"Error during cleanup: {e}", "error")

    def get_namespace(self):
        return self._ego_robot_ns
    
    def get_robot_id(self):
        return self._ego_robot_id

class MiddleWareSubscriber:
    def __init__(self):
        # Instance variables - each object gets its own
        self.context = zmq.Context.instance()
        self.sub_socket = self.context.socket(zmq.SUB)
        
        self._ego_robot_ns = rospy.get_param("ego_robot_ns", "/jackalx")
        self._ego_robot_id = extractRobotIdFromNamespace(self._ego_robot_ns)
        
        # Initialize centralized logger
        self.logger = MiddlewareLogger("ZeroMQ → ROS Subscriber", self._ego_robot_ns)
        
        # Validate robot namespace format
        if self._ego_robot_id == -999:
            self.logger.add_error("Invalid namespace format. Must start with '/' and end with robot ID")
            self.logger.log_startup_summary()
            return
        
        self.logger.add_config("Robot ID", self._ego_robot_id)
        
        # Get robot namespace list
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        self._other_robots_nss = identifyOtherRobotNamespaces(self._robot_ns_list, self._ego_robot_ns)
        
        if len(self._other_robots_nss) == 0:
            self.logger.add_warning("Empty /robot_ns_list parameter")
        else:
            self.logger.add_config("Listening to", f"{len(self._other_robots_nss)} robots: {', '.join(self._other_robots_nss)}")

        # Create ROS publishers for each other robot
        self._other_robot_ros_publisher_dict = self.create_other_robot_trajectory_ros_publishers()
        self._central_aggregator_ros_publisher = rospy.Publisher("/all_robots_reached_objective", Bool, queue_size=10)
        self._dead_man_switch_ros_publisher = rospy.Publisher("/jackal_deadman_switch", Twist, queue_size=10)
        
        self.logger.add_publication("/all_robots_reached_objective", "Bool")
        self.logger.add_publication("/jackal_deadman_switch", "Twist")
        
        # Setup ZeroMQ subscriber endpoints
        self._other_robot_subscriber_endpoints = self.get_other_robot_subscriber_endpoints()
        self._central_aggregator_endpoint      = self.get_central_aggregator_endpoint()
        self.setup_zmq_subscriber()
        
        # Flag for main loop control
        self._running = True
        
        # Log the complete startup summary once
        self.logger.log_startup_summary()

    def create_other_robot_trajectory_ros_publishers(self):
        """Create ROS publishers for each other robot's trajectory topic."""
        publisher_dict_holder = {}
    
        for other_ns in self._other_robots_nss:
            if other_ns == self._ego_robot_ns:
                continue
            
            try:
                topic_name = f"{other_ns.rstrip('/')}/robot_to_robot/output/current_trajectory"
                publisher = rospy.Publisher(topic_name, ObstacleGMM, queue_size=10)
                publisher_dict_holder[other_ns] = publisher
                self.logger.add_publication(topic_name, "ObstacleGMM")
                
            except Exception as e:
                self.logger.add_error(f"Failed to create ROS publisher for {other_ns}: {e}")
        
        return publisher_dict_holder
       
    def get_other_robot_subscriber_endpoints(self):
        """Get ZeroMQ subscriber endpoints for other robots."""
        endpoints = []
        
        for other_ns in self._other_robots_nss:
            try:
                endpoint = rospy.get_param(f"{other_ns}/network/publisher_end_point", None)
                
                if endpoint:
                    endpoints.append(endpoint)
                else:
                    self.logger.add_warning(f"No endpoint found for {other_ns}")
                    
            except Exception as e:
                self.logger.add_error(f"Failed to get endpoint for {other_ns}: {e}")
        
        return endpoints

    def get_central_aggregator_endpoint(self):
        endpoint = None
        try:
            endpoint = rospy.get_param(f"/central_aggregator/network/publisher_end_point", None)
            
            if not endpoint:
                self.logger.add_warning("No central aggregator endpoint found")
                    
        except Exception as e:
            self.logger.add_error(f"Failed to get central aggregator endpoint: {e}")
        return endpoint
        
    def setup_zmq_subscriber(self):
        """Setup ZeroMQ subscriber socket and connect to other robots."""
        try:
            # Subscribe to specific topics instead of all messages for better performance
            for other_ns in self._other_robots_nss:
                trajectory_topic = f"{other_ns.rstrip('/')}/robot_to_robot/output/current_trajectory"
                self.sub_socket.setsockopt(zmq.SUBSCRIBE, trajectory_topic.encode())
                self.logger.add_subscription(trajectory_topic, "ZMQ")
            
            # Subscribe to global topics
            global_objective_topic = "/all_robots_reached_objective"
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, global_objective_topic.encode())
            self.logger.add_subscription(global_objective_topic, "ZMQ")

            deadman_switch_topic = "/jackal_deadman_switch"
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, deadman_switch_topic.encode())
            self.logger.add_subscription(deadman_switch_topic, "ZMQ")
            
            # Connect to all other robot endpoints
            for endpoint in self._other_robot_subscriber_endpoints:
                try:
                    self.sub_socket.connect(endpoint)
                    self.logger.add_connection(endpoint, "Connected")
                except Exception as e:
                    self.logger.add_error(f"Failed to connect to {endpoint}: {e}")

            # Connect to central aggregator if endpoint exists
            if self._central_aggregator_endpoint:
                try:
                    self.sub_socket.connect(self._central_aggregator_endpoint)
                    self.logger.add_connection(self._central_aggregator_endpoint, "Central Aggregator")
                except Exception as e:
                    self.logger.add_error(f"Failed to connect to central aggregator: {e}")
                    
        except Exception as e:
            self.logger.add_error(f"Failed to setup subscriber: {e}")

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

    def _handle_deadman_switch_message(self, sender_ns, msg_bytes, metadata, topic):
        """Handle deadman switch messages."""
        try:
            deadman_msg = self.from_bytes(msg_bytes, Twist)
            
            # Determine if this is from central aggregator
            if topic == "/jackal_deadman_switch":
                # This is from the central aggregator - publish to our local deadman switch topic
                self._dead_man_switch_ros_publisher.publish(deadman_msg)
                rospy.logdebug(f"{self._ego_robot_ns}: [ZeroMQ → ROS] Relayed GLOBAL deadman_switch from central aggregator to local ROS (angular.z={deadman_msg.angular.z})")
            else:
                # This is from an individual robot - not expected for deadman switch
                rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ → ROS] Received unexpected individual deadman_switch from {sender_ns} on topic {topic} (not relaying)")
            
        except Exception as e:
            rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Failed to handle deadman_switch message: {e}")
    
    def message_processing_loop(self):
        """Main loop for processing incoming ZeroMQ messages."""
        self.logger.log_runtime_event("Processing Loop", "Started", "info")
        
        while self._running and not rospy.is_shutdown():
            try:
                # Poll with 100ms timeout to allow checking shutdown condition regularly
                if self.sub_socket.poll(timeout=0.1):
                    # Message available - receive immediately without blocking. Gets one complete multipart message from queue
                    multipart_msg = self.sub_socket.recv_multipart(zmq.NOBLOCK)
                    
                    if len(multipart_msg) != 3:
                        rospy.logerr(f"{self._ego_robot_ns}: UNEXPECTED frame count: {len(multipart_msg)}")
                        rospy.logerr(f"Frames: {[len(f) for f in multipart_msg]}")
                        rospy.logerr(f"First frame (topic?): {multipart_msg[0][:100] if len(multipart_msg) > 0 else 'N/A'}")
                        continue  # Skip malformed messages

                    # Unpack 3-frame message: [topic (filter), metadata (json), message (bytes)]
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
                        elif msg_type == "Deadman":
                            self._handle_deadman_switch_message(sender_ns, msg_bytes, metadata, topic)
                        else:
                            rospy.logwarn(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Unknown message type received from ZeroMQ: {msg_type}")
                    
            except zmq.Again:
                rospy.logerr(f"{self._ego_robot_ns}: zmq.Again")
                continue
            except Exception as e:
                rospy.logerr(f"{self._ego_robot_ns}: [ZeroMQ → ROS Subscriber] Error in message processing loop: {e}")

    def cleanup(self):
        """Cleanup resources."""
        try:
            self._running = False
            self.sub_socket.close()
            self.logger.log_runtime_event("Shutdown", "Socket closed", "info")
            
        except Exception as e:
            self.logger.log_runtime_event("Shutdown", f"Error during cleanup: {e}", "error")

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
        
        # Initialize centralized logger
        self.logger = MiddlewareLogger("Central Aggregator Publisher", "/central_aggregator")
        
        # Initialize ZeroMQ publisher socket
        self.pub_socket = self.context.socket(zmq.PUB)
        self._socket_lock = threading.Lock()  # Prevent interleaved sends
        
        # Get central aggregator endpoint configuration
        self._ego_node_end_point = rospy.get_param("/central_aggregator/network/publisher_end_point", "tcp://192.168.0.99:4000")
        self.logger.add_config("Publisher endpoint", self._ego_node_end_point)
        
        # The topic this central aggregator publishes to
        self._ego_node_all_robots_reached_objective_topic = "/all_robots_reached_objective"
        self._ego_node_jackal_deadman_switch_topic = "/jackal_deadman_switch"
        
        # Get list of robots to aggregate
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        self.logger.add_config("Robots", f"{len(self._robot_ns_list)}: {', '.join(self._robot_ns_list)}")
        
        # Bind publisher socket
        self.bind_pub_socket()
        
        # ROS subscriber to listen for local aggregation trigger
        self._local_aggregator_subscriber = rospy.Subscriber(
            self._ego_node_all_robots_reached_objective_topic,
            Bool,
            self.aggregate_callback
        )

        self._ego_node_jackal_deadman_switch_subscriber = rospy.Subscriber(
            self._ego_node_jackal_deadman_switch_topic, 
            Twist, 
            self.jackal_dead_man_switch
        )
        
        self.logger.add_subscription(self._ego_node_all_robots_reached_objective_topic, "Bool")
        self.logger.add_subscription(self._ego_node_jackal_deadman_switch_topic, "Twist")
        
        self.seq = 0
        
        # Log the complete startup summary once
        self.logger.log_startup_summary()
        
    def bind_pub_socket(self):
        """Bind ZeroMQ publisher socket."""
        try:
            self.pub_socket.bind(self._ego_node_end_point)
            self.logger.add_connection(self._ego_node_end_point, "Bound")
        except Exception as e:
            self.logger.add_error(f"Failed to bind: {e}")
    
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
            
            with self._socket_lock:  # Atomic multipart send
                self.pub_socket.send_multipart([
                    self._ego_node_all_robots_reached_objective_topic.encode(),
                    json.dumps(meta_data).encode(),
                    self.to_bytes(msg)
                ])
            
            self.logger.log_runtime_event("ROS → ZeroMQ", f"Relayed global objective_reached={msg.data}", "info")
            
        except Exception as e:
            self.logger.log_runtime_event("ROS → ZeroMQ", f"Failed to relay: {e}", "error")
    
    def jackal_dead_man_switch(self, msg):
        try:
            self.seq += 1
            
            meta_data = {
                "type": "Deadman",
                "seq": self.seq,
                "stamp": rospy.Time.now().to_sec(),
                "from": "/central_aggregator"
            }
            
            with self._socket_lock:  # Atomic multipart send
                self.pub_socket.send_multipart([
                    self._ego_node_jackal_deadman_switch_topic.encode(),
                    json.dumps(meta_data).encode(),
                    self.to_bytes(msg)
                ])
            
            rospy.logdebug(f"[Central Aggregator] Relayed deadman_switch (angular.z={msg.angular.z})")
            
        except Exception as e:
            self.logger.log_runtime_event("ROS → ZeroMQ", f"Failed to relay deadman_switch: {e}", "error")

    def cleanup(self):
        """Cleanup resources."""
        try:
            self.pub_socket.close()
            self.logger.log_runtime_event("Shutdown", "Socket closed", "info")
        except Exception as e:
            self.logger.log_runtime_event("Shutdown", f"Error during cleanup: {e}", "error")

class MiddleWareCentralAggBaseSub(MiddleWareCentralAggBase):
    def __init__(self):
        super().__init__()
        
        # Initialize centralized logger
        self.logger = MiddlewareLogger("Central Aggregator Subscriber", "/central_aggregator")
        
        # Initialize ZeroMQ subscriber socket
        self.sub_socket = self.context.socket(zmq.SUB)
        
        # Get robot namespace list
        self._robot_ns_list = rospy.get_param("/robot_ns_list", [])
        self.logger.add_config("Robots", f"{len(self._robot_ns_list)}: {', '.join(self._robot_ns_list)}")
        
        self._other_robot_subscriber_endpoints = self.get_robot_endpoints()
        
        # Create ROS publishers for each robot's objective reached topic
        self._robots_objective_reached_publisher_dict = self.create_robot_objective_publishers()
        
        # Setup ZeroMQ subscriptions
        self.setup_zmq_subscriber()
        
        # Flag for main loop control
        self._running = True
        
        # Log the complete startup summary once
        self.logger.log_startup_summary()
    
    def get_robot_endpoints(self):
        """Get endpoints for all robots."""
        endpoints = []
        for robot_ns in self._robot_ns_list:
            try:
                endpoint = rospy.get_param(f"{robot_ns}/network/publisher_end_point", None)
                if endpoint:
                    endpoints.append(endpoint)
            except Exception as e:
                self.logger.add_error(f"Failed to get endpoint for {robot_ns}: {e}")
        return endpoints
    
    def create_robot_objective_publishers(self):
        """Create ROS publishers for each robot's objective reached topic."""
        publisher_dict = {}
        for robot_ns in self._robot_ns_list:
            try:
                topic_name = f"{robot_ns.rstrip('/')}/events/objective_reached"
                publisher = rospy.Publisher(topic_name, Bool, queue_size=10)
                publisher_dict[robot_ns] = publisher
                self.logger.add_publication(topic_name, "Bool")
            except Exception as e:
                self.logger.add_error(f"Failed to create ROS publisher for {robot_ns}: {e}")
        return publisher_dict
    
    def setup_zmq_subscriber(self):
        """Setup ZeroMQ subscriber."""
        try:
            for robot_ns in self._robot_ns_list:
                # Extract robot ID using the same function as publisher
                robot_id = extractRobotIdFromNamespace(robot_ns)
                objective_topic = f"{robot_id}{robot_id}{robot_id}{robot_ns.rstrip('/')}/events/objective_reached"
                self.sub_socket.setsockopt(zmq.SUBSCRIBE, objective_topic.encode())
                self.logger.add_subscription(objective_topic, "ZMQ")
                
                try:
                    endpoint = rospy.get_param(f"{robot_ns}/network/publisher_end_point", None)
                    if endpoint:
                        self.sub_socket.connect(endpoint)
                        self.logger.add_connection(endpoint, robot_ns)
                        time.sleep(0.1)
                    else:
                        self.logger.add_warning(f"No endpoint found for {robot_ns}")
                except Exception as e:
                    self.logger.add_error(f"Failed to connect to {robot_ns}: {e}")
                time.sleep(0.1)
        except Exception as e:
            self.logger.add_error(f"Setup failed: {e}")
    
    def message_processing_loop(self):
        """Process incoming robot objective messages and relay to ROS."""
        while self._running and not rospy.is_shutdown():
            try:
                # Poll with 100ms timeout to allow checking shutdown condition regularly
                if self.sub_socket.poll(timeout=0.1):
                    multipart_msg = self.sub_socket.recv_multipart(zmq.NOBLOCK)
                    

                    if len(multipart_msg) != 3:
                        rospy.logerr(f"[Central Aggregator]: UNEXPECTED frame count: {len(multipart_msg)}")
                        rospy.logerr(f"Frames: {[len(f) for f in multipart_msg]}")
                        rospy.logerr(f"First frame (topic?): {multipart_msg[0][:100] if len(multipart_msg) > 0 else 'N/A'}")
                        # Option 3: Use repr() to safely show bytes
                        for i, m in enumerate(multipart_msg):
                            print(f"Frame {i}: {m[:100]}")  # First 100 bytes as raw bytes
                        continue  # Skip malformed messages

                    if len(multipart_msg) == 3:
                        topic_bytes, metadata_bytes, msg_bytes = multipart_msg
                        
                        topic = topic_bytes.decode()
                        
                        # CRITICAL: Only process objective_reached messages, ignore trajectory messages
                        if "/events/objective_reached" not in topic:
                            rospy.logdebug(f"[Central Aggregator ZeroMQ → ROS] Ignoring non-objective message on topic: {topic}")
                            continue
                        
                        metadata = json.loads(metadata_bytes.decode())
                        sender_ns = metadata.get("from", "unknown")
                        msg_type = metadata.get("type", "unknown")
                        
                        # Verify it's a Bool message (objective_reached should always be Bool)
                        if msg_type != "Bool":
                            rospy.logwarn_throttle(5.0, f"[Central Aggregator ZeroMQ → ROS Subscriber] "
                                                    f"Received unexpected message type '{msg_type}' from {sender_ns} "
                                                    f"on topic {topic}. Expected 'Bool'. Ignoring...")
                            continue
                        
                        # Deserialize the Bool message
                        objective_msg = self.from_bytes(msg_bytes, Bool)
                        
                        # Simply republish to local ROS topic
                        if sender_ns in self._robots_objective_reached_publisher_dict:
                            self._robots_objective_reached_publisher_dict[sender_ns].publish(objective_msg)
                            rospy.loginfo(f"[Central Aggregator ZeroMQ → ROS] Relayed objective_reached from {sender_ns} to local ROS topic, status: {objective_msg.data}")
                        else:
                            rospy.logwarn(f"[Central Aggregator ZeroMQ → ROS] Received objective_reached from unknown robot {sender_ns}")
                            
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
            self.sub_socket.close()
            self.logger.log_runtime_event("Shutdown", "Socket closed", "info")
            
        except Exception as e:
            self.logger.log_runtime_event("Shutdown", f"Error during cleanup: {e}", "error")