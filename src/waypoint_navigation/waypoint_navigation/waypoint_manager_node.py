import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


# Helper function to convert Yaw (in radians) to a ROS Quaternion
def euler_to_quaternion(yaw):
    """Converts a yaw angle (in radians) into a Quaternion message."""
    # Since pitch and roll are 0 for 2D navigation, we only use yaw.
    q = quaternion_from_euler(0, 0, yaw)
    quaternion = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return quaternion


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")

        # 1. DECLARE & GET PARAMETER (Path to waypoints.yaml)
        self.declare_parameter("waypoint_config_file", "")
        config_path = (
            self.get_parameter("waypoint_config_file")
            .get_parameter_value()
            .string_value
        )

        # 2. Load Waypoints
        if not os.path.exists(config_path):
            self.get_logger().error(f"Waypoint config file not found at: {config_path}")
            raise FileNotFoundError(f"Missing required config file: {config_path}")

        with open(config_path, "r") as f:
            self.waypoints = yaml.safe_load(f)["waypoints"]
            self.get_logger().info(
                f"Successfully loaded {len(self.waypoints)} waypoints."
            )

        # 3. Action Client and State Management
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_handle = None
        self._nav_active = threading.Event()  # Used to check if nav is running
        self.waypoint_index = 0
        self.last_feedback_time = self.get_clock().now()

        # 4. ROS Subscriptions and Publications
        # FIX 1: Changed subscription from "/waypoint_queue" to "waypoint_queue"
        self.create_subscription(
            String, "waypoint_queue", self.waypoint_queue_callback, 10
        )
        self.status_publisher = self.create_publisher(String, "/navigation_status", 10)

        self.get_logger().info("Waypoint Manager Node running and listening for queue.")

        self.current_waypoint_queue = []

        # 5. Marker Publisher (RViz)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 10
        )
        self.create_timer(1.0, self.publish_waypoint_markers)

        # FIX 2: Removed erroneous self.get_logger.info("...") call.

    # ------------------------------------------------------------------------------
    # ROS 2 Callbacks (Run in the main ROS thread)
    # ------------------------------------------------------------------------------

    def waypoint_queue_callback(self, msg):
        """
        Processes a comma-separated string of waypoint names received from the GUI.
        """
        # The msg.data will be a string like: "station_a,station_b,home"
        waypoint_names = [
            name.strip().lower()  # Normalize name
            for name in msg.data.split(",")
            if name.strip().lower() in self.waypoints
        ]

        if not waypoint_names:
            self.get_logger().warn("Received empty or invalid waypoint queue.")
            return

        # Add 'home' as the final destination if it's not already the last one
        if waypoint_names[-1] != "home" and "home" in self.waypoints:
            waypoint_names.append("home")

        self.current_waypoint_queue = waypoint_names
        self.get_logger().info(f"Received new queue: {self.current_waypoint_queue}")

        # Check if navigation is currently active using the threading.Event
        if self._nav_active.is_set():
            self.get_logger().warn(
                "Navigation is already in progress. Ignoring new queue for now."
            )
            return

        # Start the navigation sequence in a separate thread to keep the ROS 2 executor spinning
        self._nav_active.set()  # Set the flag to active
        self.waypoint_index = 0
        self.navigate_sequence()

    # ------------------------------------------------------------------------------
    # Action Client Logic (Run in a separate thread)
    # ------------------------------------------------------------------------------

    def navigate_sequence(self):
        """
        Non-blocking driver that sends the next goal in the queue.
        The sequence is chained using callbacks (see get_result_callback).
        """
        if not self._nav_active.is_set():
            self.get_logger().warn(
                "Navigation sequence externally cancelled or complete."
            )
            return

        if self.waypoint_index >= len(self.current_waypoint_queue):
            # All waypoints completed
            self.get_logger().info("Waypoint sequence complete, robot is Home.")
            self.status_publisher.publish(
                String(data="Sequence Complete, Home Reached")
            )
            self._nav_active.clear()  # Reset flag after successful sequence
            return

        wp_name = self.current_waypoint_queue[self.waypoint_index]

        self.get_logger().info(
            f"Starting navigation to {wp_name} ({self.waypoint_index + 1}/{len(self.current_waypoint_queue)})"
        )

        status_msg = String(data=f"Navigating... to {wp_name}")
        self.status_publisher.publish(status_msg)

        # 1. Create Goal Message
        wp_data = self.waypoints[wp_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = wp_data["x"]
        goal_msg.pose.pose.position.y = wp_data["y"]
        yaw_rad = wp_data.get("yaw", 0.0)
        goal_msg.pose.pose.orientation = euler_to_quaternion(yaw_rad)

        # 2. Wait for Action Server (Non-blocking check)
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("NavigateToPose action server not available!")
            self._nav_active.clear()
            self.status_publisher.publish(String(data="Failed: Server Not Ready"))
            return

        # 3. Send Goal (Non-blocking)
        # 💡 IMPORTANT: Goal response is handled by a callback, freeing the executor
        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """
        Receives and processes real-time navigation feedback with throttling.
        """
        # Define the minimum interval (e.g., 0.5 seconds = 2 Hz)
        THROTTLE_INTERVAL_SEC = 0.5

        current_time = self.get_clock().now()
        time_diff = current_time - self.last_feedback_time

        # 💡 THROTTLING LOGIC: Only process if the interval has passed
        if time_diff.nanoseconds / 1e9 < THROTTLE_INTERVAL_SEC:
            return  # Skip processing this feedback message

        # Update the last publication time
        self.last_feedback_time = current_time

        # --- Normal Feedback Processing (Only runs if not throttled) ---

        distance_remaining = feedback_msg.feedback.distance_remaining

        # Determine the name of the current waypoint
        if self.waypoint_index < len(self.current_waypoint_queue):
            current_wp_name = self.current_waypoint_queue[self.waypoint_index]
        else:
            current_wp_name = "Unknown"

        # Format and publish a status update (only occurs every 0.5 seconds)
        status_text = (
            f"Navigating to {current_wp_name}... " f"Dist: {distance_remaining:.2f} m"
        )

        self.status_publisher.publish(String(data=status_text))
        self.get_logger().debug(f"Throttled Feedback Published: {status_text}")

    def goal_response_callback(self, future):
        """Handles the response after the goal is sent."""
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            wp_name = self.current_waypoint_queue[self.waypoint_index]
            self.get_logger().error(f"Goal to {wp_name} rejected by server.")
            self._nav_active.clear()
            self.status_publisher.publish(
                String(data=f"Failed: Goal to {wp_name} Rejected")
            )
            return

        # 💡 IMPORTANT: Wait for result (Non-blocking)
        # Result future is handled by the next callback, freeing the executor
        self._goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handles the final result after the goal is completed."""
        result = future.result()
        current_wp_name = self.current_waypoint_queue[self.waypoint_index]
        self._goal_handle = None

        # 1. Check Result Status
        if result.status == 4:  # GoalStatus.SUCCEEDED
            self.get_logger().info(f"Successfully reached {current_wp_name}")
            self.status_publisher.publish(String(data=f"Reached: {current_wp_name}"))

            # Move to the next waypoint
            self.waypoint_index += 1
            self.navigate_sequence()  # 💡 RECURSIVE CALL: Starts the next goal
        else:
            # Goal failed
            self.get_logger().error(
                f"Navigation to {current_wp_name} failed with status: {result.status}"
            )
            self.status_publisher.publish(
                String(data=f"Failed: Navigation to {current_wp_name}")
            )
            self.stop_navigation()

    def stop_navigation(self):
        """
        Cancels the current active navigation goal and stops the sequence.
        """
        self._nav_active.clear()  # Clear the event to stop the thread loop

        if self._action_client.server_is_ready() and self._goal_handle:
            self.get_logger().info("Cancelling current navigation goal...")
            # We don't need to block on the cancel, as the thread will exit shortly
            self._action_client.cancel_goal_async(self._goal_handle)
            self._goal_handle = None
            self.status_publisher.publish(String(data="Cancelled by User"))
        else:
            self.get_logger().warn(
                "No active goal or action server not ready to cancel."
            )
            self.status_publisher.publish(String(data="Navigation Stopped"))

    def publish_waypoint_markers(self):
        """Generates a MarkerArray for all waypoints and publishes it to RViz."""
        marker_array = MarkerArray()

        # Optional: Delete all previous markers to ensure a clean display
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = "map"
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)

        id_counter = 0

        for name, data in self.waypoints.items():
            # --- 1. Position Marker (Sphere or Cube) ---
            position_marker = Marker()
            position_marker.header.frame_id = "map"
            position_marker.header.stamp = self.get_clock().now().to_msg()
            position_marker.ns = "waypoints_position"
            position_marker.id = id_counter
            position_marker.type = Marker.SPHERE
            position_marker.action = Marker.ADD

            position_marker.pose.position.x = data["x"]
            position_marker.pose.position.y = data["y"]
            position_marker.pose.position.z = 0.1  # Lift slightly above floor

            position_marker.scale.x = 0.2
            position_marker.scale.y = 0.2
            position_marker.scale.z = 0.2

            # Color based on name (e.g., green for 'home', blue for others)
            color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Default Blue
            if name == "home":
                color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green for Home

            position_marker.color = color
            marker_array.markers.append(position_marker)
            id_counter += 1

            # --- 2. Text Marker (Waypoint Label) ---
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoints_label"
            text_marker.id = id_counter
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = data["x"]
            text_marker.pose.position.y = data["y"]
            text_marker.pose.position.z = 0.5  # Position text above the sphere

            text_marker.scale.z = 0.3  # Height of the text
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White text
            text_marker.text = name.upper()

            marker_array.markers.append(text_marker)
            id_counter += 1

        # Publish the combined marker array
        self.marker_publisher.publish(marker_array)
        self.get_logger().info(
            f"Published {len(marker_array.markers)} markers to RViz."
        )


def main(args=None):
    rclpy.init(args=args)

    # 🚨 Using MultiThreadedExecutor is safer when using internal threads
    executor = MultiThreadedExecutor()

    try:
        waypoint_manager = WaypointManager()
        executor.add_node(waypoint_manager)

        # Spin the executor to process callbacks (subscriptions and action client responses)
        executor.spin()

    except FileNotFoundError as e:
        # Check if waypoint_manager was created before trying to access its logger
        if "waypoint_manager" in locals():
            waypoint_manager.get_logger().fatal(f"FATAL ERROR: {e}")
        else:
            print(f"FATAL ERROR: {e}")
    except Exception as e:
        if "waypoint_manager" in locals():
            waypoint_manager.get_logger().fatal(f"An unexpected error occurred: {e}")
        else:
            print(f"An unexpected error occurred: {e}")
    finally:
        if "waypoint_manager" in locals():
            waypoint_manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
