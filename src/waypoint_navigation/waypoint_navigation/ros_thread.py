import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
# Need all these for a robust ROS 2 thread implementation
from std_msgs.msg import String as StringMsg


class ROS2NodeThread(QThread):
    """
    A separate thread to run the ROS 2 event loop (executor), preventing the
    GUI from freezing. It contains all ROS 2 communication logic.
    """

    status_signal = pyqtSignal(str)  # For general GUI status updates
    ready_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._running = True
        # Ensure rclpy is initialized once
        if not rclpy.ok():
            rclpy.init()

        self.node = None
        self.executor = None
        self.amcl_ready = False

    def run(self):
        # Create Node and Executor
        self.node = Node("waypoint_gui_node_thread")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        # 1. Waypoint Queue Publisher
        self.waypoint_publisher = self.node.create_publisher(
            StringMsg,
            "waypoint_queue",
            10,
        )

        # 2. Command Publisher (NEW: For STOP/CANCEL)
        self.command_publisher = self.node.create_publisher(
            StringMsg,
            "navigation_command",  # This topic receives the 'STOP' command
            10,
        )

        # 3. Status Subscriber
        self.status_subscriber = self.node.create_subscription(
            StringMsg,
            "/navigation_status",
            self.navigation_status_callback,
            10,
        )

        # 4. AMCL Pose Subscriber (for initial readiness check)
        self.amcl_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self.amcl_callback, 1
        )

        self.node.get_logger().info("ROS Node Thread started. Waiting for AMCL pose...")
        self.status_signal.emit("ROS Node Thread started. Waiting for AMCL pose...")

        # Use executor spin loop for non-blocking operation
        while rclpy.ok() and self._running:
            self.executor.spin_once(timeout_sec=0.01)

        # Cleanup
        if self.node:
            self.node.destroy_node()

    def stop(self):
        self._running = False
        self.node.get_logger().info("Stopping ROS 2 Thread...")

    def navigation_status_callback(self, msg):
        """Passes status updates from the WaypointManager to the GUI."""
        self.status_signal.emit(msg.data)

    def amcl_callback(self, msg):
        """Checks if AMCL is publishing, indicating the navigation stack is up."""
        if not self.amcl_ready:
            self.amcl_ready = True
            self.node.get_logger().info("AMCL pose received. System considered ready.")
            self.status_signal.emit("System Ready. Navigation controls enabled.")
            self.ready_signal.emit()
            # Unsubscribe after initial check to save resources
            self.node.destroy_subscription(self.amcl_subscriber)

    def publish_waypoint_queue(self, queue):
        """
        Publishes the list of waypoints as a single comma-separated string.
        """
        if not self.node or not queue:
            self.node.get_logger().warn("Node not ready or empty queue.")
            return

        queue_string = ",".join(queue)
        self.node.get_logger().info(f'Publishing new waypoint queue: "{queue_string}"')

        msg = StringMsg()
        msg.data = queue_string
        self.waypoint_publisher.publish(msg)

        self.status_signal.emit(f"Queue Sent ({len(queue)} items)")

    def cancel_navigation(self):
        """
        Sends a 'STOP' command to the WaypointManager on the 'navigation_command' topic.
        """
        if not self.node:
            print("ROS node not initialized.")
            return

        command = "STOP"
        msg = StringMsg(data=command)
        self.command_publisher.publish(msg)

        self.node.get_logger().warn(f"Published navigation command: {command}")
        self.status_signal.emit(f"Command Sent: {command}")
