import time

import rclpy
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.node import Node
from std_msgs.msg import String as StringMsg


class ROS2NodeThread(QThread):
    status_signal = pyqtSignal(str)  # For general GUI status updates
    ready_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._running = True

    def run(self):
        rclpy.init()
        self.node = Node("waypoint_gui_node_thread")

        # 1. Create Publisher for the Queue
        self.waypoint_publisher = self.node.create_publisher(
            StringMsg,
            "waypoint_queue",
            10,
        )

        # 2. Create Subscriber for Navigation Status
        self.status_subscriber = self.node.create_subscription(
            StringMsg,
            "/navigation_status",
            self.navigation_status_callback,
            10,
        )

        self.status_signal.emit("ROS node started")

        # Wait for the ROS stack (Gazebo/Nav2) to settle
        time.sleep(0.5)
        self.status_signal.emit("System Ready")
        self.ready_signal.emit()

        while self._running:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        self._running = False

    def navigation_status_callback(self, msg):
        """Passes status updates from the WaypointManager to the GUI."""
        self.status_signal.emit(msg.data)

    # 🛑 FIXED PUBLISHING LOGIC
    def publish_waypoint_queue(self, queue):
        """
        Publishes the list of waypoints as a single comma-separated string,
        as expected by the WaypointManager.
        """
        if not queue:
            self.node.get_logger().warn("Attempted to publish an empty queue.")
            return

        # 1. Join the list of waypoint names into a single comma-separated string
        queue_string = ",".join(queue)
        self.node.get_logger().info(f'Publishing new waypoint queue: "{queue_string}"')

        # 2. Create and publish the single String message
        msg = StringMsg()
        msg.data = queue_string
        self.waypoint_publisher.publish(msg)

        # Signal the GUI that the action was performed
        self.status_signal.emit(f"Queue Sent ({len(queue)} items)")

    def cancel_navigation(self):
        """
        Sends a cancellation signal (e.g., a specific string) to the WaypointManager.

        NOTE: Since your WaypointManager doesn't have a dedicated 'cancel' subscriber,
        we'll log the intention here. For full functionality, you'd need to add a
        cancel subscription/service/action client to the ROS2NodeThread.
        """
        self.node.get_logger().warn(
            "Cancellation signal sent (Requires dedicated subscriber in WaypointManager)"
        )
        self.status_signal.emit("Cancellation Requested (GUI only)")
        # In a real system, you would publish to a dedicated topic like:
        # cancel_msg = StringMsg(data="CANCEL")
        # self.cancel_publisher.publish(cancel_msg)
