import sys
import time

from PyQt5.QtCore import QTimer  # Import QTimer for better time handling
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QLabel, QListWidget,
                             QListWidgetItem, QPushButton, QVBoxLayout,
                             QWidget)

from waypoint_navigation.ros_thread import ROS2NodeThread


class WaypointGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TurtleBot3 Waypoint Commander")
        self.setGeometry(100, 100, 400, 550)
        self.init_ui()
        self.init_ros()
        self.set_navigation_enabled(False)
        self.ros_time_timer = QTimer(self)
        self.ros_time_timer.timeout.connect(self.update_current_ros_time)
        self.current_ros_time_sec = time.time()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # --- 1. Available Waypoint Buttons ---
        self.waypoint_names = [
            "station_a",
            "station_b",
            "station_c",
            "station_d",
            "docking_station",
            "home",
        ]

        main_layout.addWidget(QLabel("Available Waypoints:"))

        # Waypoint selection buttons layout (2 columns)
        button_container = QWidget()
        button_container_layout = QVBoxLayout(button_container)

        current_row_layout = QHBoxLayout()
        for i, name in enumerate(self.waypoint_names):
            btn = QPushButton(name.replace("_", " ").title())
            btn.clicked.connect(lambda _, n=name: self.add_waypoint_to_queue(n))
            current_row_layout.addWidget(btn)
            if (i + 1) % 3 == 0 or i == len(self.waypoint_names) - 1:
                button_container_layout.addLayout(current_row_layout)
                current_row_layout = QHBoxLayout()

        main_layout.addWidget(button_container)

        # --- 2. Waypoint Queue and Selection ---
        self.queue_list = QListWidget()
        self.queue_list.setMaximumHeight(150)
        main_layout.addWidget(QLabel("Selected Waypoint Queue:"))
        main_layout.addWidget(self.queue_list)

        # --- 3. Navigation Control Buttons ---
        control_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Navigation")
        self.start_btn.clicked.connect(self.start_navigation)
        control_layout.addWidget(self.start_btn)

        self.clear_btn = QPushButton("Clear Queue")
        self.clear_btn.clicked.connect(self.queue_list.clear)
        control_layout.addWidget(self.clear_btn)

        self.stop_btn = QPushButton("Stop/Cancel Navigation")
        self.stop_btn.clicked.connect(self.stop_navigation)
        control_layout.addWidget(self.stop_btn)

        main_layout.addLayout(control_layout)

        # --- 4. Status Display ---
        self.status_label = QLabel("Navigation Status: Initializing ROS...")
        main_layout.addWidget(self.status_label)

        self.logs_widget = QListWidget()
        self.logs_widget.setMaximumHeight(150)
        main_layout.addWidget(QLabel("Navigation Status Logs:"))
        main_layout.addWidget(self.logs_widget)

        self.setLayout(main_layout)

    def init_ros(self):
        self.ros_thread = ROS2NodeThread()
        self.ros_thread.status_signal.connect(self.update_status)
        self.ros_thread.ready_signal.connect(self.handle_amcl_ready)
        self.ros_thread.start()

    def set_navigation_enabled(self, enabled):
        """Enable/disable all navigation-related buttons."""
        self.start_btn.setEnabled(enabled)
        self.clear_btn.setEnabled(enabled)
        for child in self.findChildren(QPushButton):
            if child not in [self.start_btn, self.clear_btn, self.stop_btn]:
                child.setEnabled(enabled)

    def handle_amcl_ready(self):
        """Called when ROS 2 stack confirms AMCL/Nav2 is ready."""
        self.status_label.setText(
            "Navigation Status: System Ready. Select waypoints to begin."
        )
        self.set_navigation_enabled(True)
        self.ros_time_timer.start(100)  # Update every 100ms

    def update_current_ros_time(self):
        """Updates the internal ROS time representation."""
        if self.ros_thread.node:
            try:
                # Get the time in nanoseconds and convert to seconds
                time_nanosec = self.ros_thread.node.get_clock().now().nanoseconds
                self.current_ros_time_sec = time_nanosec / 1e9
            except Exception:
                # Fallback to system time if ROS time is unavailable
                self.current_ros_time_sec = time.time()
        else:
            self.current_ros_time_sec = time.time()

    def add_waypoint_to_queue(self, name):
        # The stored item text needs to match the key name for retrieval in start_navigation
        display_name = name.replace("_", " ").title()
        QListWidgetItem(display_name, self.queue_list)

    def start_navigation(self):
        waypoint_queue = []
        for i in range(self.queue_list.count()):
            display_name = self.queue_list.item(i).text()
            key_name = display_name.lower().replace(" ", "_")
            if key_name in self.waypoint_names:
                waypoint_queue.append(key_name)

        if waypoint_queue:
            self.ros_thread.publish_waypoint_queue(waypoint_queue)
            self.status_label.setText("Navigation Status: Queue Being Processed...")
        else:
            self.status_label.setText("Navigation Status: Queue Empty!")

    def update_status(self, status_msg):
        """Updates the status log from signals received from the ROS thread."""

        # Format the time using the last known ROS time
        time_str = (
            f"[{time.strftime('%H:%M:%S', time.localtime(self.current_ros_time_sec))}]"
        )

        # Append the message to the logs
        self.logs_widget.addItem(f"{time_str} {status_msg}")
        self.logs_widget.scrollToBottom()
        self.status_label.setText(f"Navigation Status: {status_msg}")

    def stop_navigation(self):
        """Requests the ROS thread to send a cancellation signal."""
        self.ros_thread.cancel_navigation()
        self.status_label.setText("Navigation Status: Cancellation requested.")

    def closeEvent(self, event):
        self.ros_time_timer.stop()
        self.ros_thread.stop()
        self.ros_thread.wait()
        event.accept()


def main_gui():
    app = QApplication(sys.argv)
    gui = WaypointGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main_gui()
