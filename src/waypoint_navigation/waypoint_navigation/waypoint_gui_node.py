import sys
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QLabel, QListWidget,
                             QListWidgetItem, QPushButton, QVBoxLayout,
                             QWidget)

# Assuming ros_thread.py is in the correct package structure
from waypoint_navigation.ros_thread import ROS2NodeThread


class WaypointGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TurtleBot3 Waypoint Commander")
        self.setGeometry(100, 100, 400, 500)
        self.init_ui()
        self.init_ros()

        # Initial state: Navigation controls disabled until AMCL is ready
        self.set_navigation_enabled(False)

    def init_ui(self):
        main_layout = QVBoxLayout()

        # --- 1. Placeholder for layout ---
        loc_layout = QHBoxLayout()
        main_layout.addLayout(loc_layout)

        # --- 2. Waypoint Queue and Selection ---
        self.waypoint_names = [
            "station_a",
            "station_b",
            "station_c",
            "station_d",
            "docking_station",
            "home",
        ]  # Matches YAML keys

        self.queue_list = QListWidget()
        main_layout.addWidget(QLabel("Selected Waypoint Queue:"))
        main_layout.addWidget(self.queue_list)

        # Waypoint selection buttons
        button_layout = QHBoxLayout()
        for name in self.waypoint_names:
            btn = QPushButton(name.replace("_", " ").title())
            # Fix lambda to ensure correct name binding
            btn.clicked.connect(lambda _, n=name: self.add_waypoint_to_queue(n))
            button_layout.addWidget(btn)
        main_layout.addLayout(button_layout)

        # --- 3. Navigation Control Buttons ---
        control_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Navigation")
        self.start_btn.clicked.connect(self.start_navigation)
        control_layout.addWidget(self.start_btn)

        self.clear_btn = QPushButton("Clear Queue")
        self.clear_btn.clicked.connect(self.queue_list.clear)
        control_layout.addWidget(self.clear_btn)

        self.stop_btn = QPushButton("Stop/Cancel Navigation")
        # 🛑 CONNECT TO THE NEW STOP FUNCTION
        self.stop_btn.clicked.connect(self.stop_navigation)
        control_layout.addWidget(self.stop_btn)

        main_layout.addLayout(control_layout)

        # --- 4. Status Display ---
        self.logs_widget = QListWidget()  # 👈 Use a QListWidget for a clean log
        self.logs_widget.setMaximumHeight(150)  # Limit the size
        main_layout.addWidget(QLabel("Navigation Status Logs:"))
        main_layout.addWidget(self.logs_widget)
        self.status_label = QLabel("Navigation Status: Initializing ROS...")
        main_layout.addWidget(self.status_label)

        self.setLayout(main_layout)

    def init_ros(self):
        self.ros_thread = ROS2NodeThread()
        # 🛑 status_signal will now receive updates from both ros_thread and waypoint_manager
        self.ros_thread.status_signal.connect(self.update_status)
        self.ros_thread.ready_signal.connect(self.handle_amcl_ready)
        self.ros_thread.start()

    def set_navigation_enabled(self, enabled):
        """Enable/disable all navigation-related buttons."""
        self.start_btn.setEnabled(enabled)
        self.clear_btn.setEnabled(enabled)
        self.stop_btn.setEnabled(enabled)
        for child in self.findChildren(QPushButton):
            # Only enable waypoint selection buttons if the main controls are enabled
            if child not in [self.start_btn, self.clear_btn, self.stop_btn]:
                child.setEnabled(enabled)

    def handle_amcl_ready(self):
        """Called when ROS 2 stack confirms AMCL/Nav2 is ready."""
        self.status_label.setText(
            "Navigation Status: System Ready. Initial pose sent automatically."
        )
        self.set_navigation_enabled(True)

    def add_waypoint_to_queue(self, name):
        # The stored item text needs to match the key name for retrieval in start_navigation
        display_name = name.replace("_", " ").title()
        QListWidgetItem(display_name, self.queue_list)

    def start_navigation(self):
        waypoint_queue = []
        for i in range(self.queue_list.count()):
            display_name = self.queue_list.item(i).text()
            # Convert the displayed name back to the internal key name
            key_name = display_name.lower().replace(" ", "_")
            if key_name in self.waypoint_names:
                waypoint_queue.append(key_name)

        if waypoint_queue:
            self.ros_thread.publish_waypoint_queue(waypoint_queue)
            self.status_label.setText("Navigation Status: Queue Being Processed...")
        else:
            self.status_label.setText("Navigation Status: Queue Empty!")

    # In WaypointGUI class:

    def update_status(self, status_msg):
        """Updates the status log from signals received from the ROS thread."""

        # 🛑 FIX: Correctly convert the ROS 2 Time object to seconds.
        # ROS 2 time objects have a .nanoseconds attribute, which you convert to seconds.

        # Get the time in nanoseconds
        time_nanosec = self.ros_thread.node.get_clock().now().nanoseconds

        # Convert nanoseconds to seconds (float)
        current_time_sec = time_nanosec / 1e9

        # Format the time
        time_str = f"[{time.strftime('%H:%M:%S', time.localtime(current_time_sec))}]"

        # Append the message to the logs
        self.logs_widget.addItem(f"{time_str} {status_msg}")
        self.logs_widget.scrollToBottom()  # Auto-scroll to the newest message

        # Also update the primary status label (optional, but good for summary)
        self.status_label.setText(f"Navigation Status: {status_msg}")

    # 🛑 NEW STOP NAVIGATION LOGIC
    def stop_navigation(self):
        """Requests the ROS thread to send a cancellation signal."""
        self.ros_thread.cancel_navigation()
        self.status_label.setText(
            "Navigation Status: Cancellation requested (Check Console)"
        )

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        event.accept()


def main_gui():
    app = QApplication(sys.argv)
    gui = WaypointGUI()
    gui.show()
    sys.exit(app.exec_())
