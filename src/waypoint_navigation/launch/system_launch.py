import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  TextSubstitution)


def generate_launch_description():
    # --- 1. SETUP PATHS AND ARGUMENTS ---
    pkg_turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_waypoint_navigation = get_package_share_directory("waypoint_navigation")

    # Define absolute paths for configuration files
    waypoints_config_path = os.path.join(
        pkg_waypoint_navigation, "config", "waypoints.yaml"
    )
    nav2_params_file = os.path.join(pkg_nav2_bringup, "params", "nav2_params.yaml")

    # 1. DECLARE ARGUMENT FOR MAP PATH
    map_file_path = os.path.join(pkg_waypoint_navigation, "map", "map.yaml")
    map_yaml_arg = DeclareLaunchArgument(
        "map",
        default_value=TextSubstitution(text=map_file_path),
        description="Full path to map yaml file to load",
    )
    map_yaml_config = LaunchConfiguration("map")

    # --- DEBUGGING LOG ---
    log_map_path = LogInfo(msg=["Attempting to load map from path: ", map_yaml_config])

    # --- 2. LAUNCH GAZEBO & NAV2 ---

    # Launch Gazebo world (using the default TurtleBot3 world)
    start_world_cmd = IncludeLaunchDescription(
        PathJoinSubstitution(
            [pkg_turtlebot3_gazebo, "launch", "turtlebot3_world.launch.py"]
        )
    )

    # Launch Nav2 stack components (includes the main lifecycle manager)
    nav2_bringup_cmd = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_nav2_bringup, "launch", "bringup_launch.py"]),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": nav2_params_file,
            "map": TextSubstitution(text=""),
            "autostart": "True",
        }.items(),
    )

    # --- 3. EXPLICIT MAP SERVER ---
    map_server_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml_config}],
        namespace="",
    )

    # --- 4. CUSTOM NODES ---

    # Waypoint Manager Node (Action Client)
    waypoint_manager_node = Node(
        package="waypoint_navigation",
        executable="waypoint_manager_node",
        name="waypoint_manager",
        output="screen",
        parameters=[{"waypoint_config_file": waypoints_config_path}],
    )

    # GUI Node (PyQt5 App)
    gui_node = Node(
        package="waypoint_navigation",
        executable="waypoint_gui_node",
        name="waypoint_gui",
        output="screen",
    )

    # --- 5. RViz Visualization ---

    rviz_config_dir = PathJoinSubstitution(
        [pkg_nav2_bringup, "rviz", "nav2_default_view.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        output="screen",
        arguments=["-d", rviz_config_dir],
    )

    # Auto-initial pose node
    auto_init_node = Node(
        package="waypoint_navigation",
        executable="auto_initial_pose_node",
        name="auto_initial_pose_node",
        output="screen",
        parameters=[
            {
                "use_gazebo_pose": False,
                "default_x": -1.999942,
                "default_y": -0.500001,
                "default_yaw": 0.0,
                "robot_name": "turtlebot3_burger",
            }
        ],
    )

    # --- 6. RETURN LAUNCH DESCRIPTION ---

    return LaunchDescription(
        [
            map_yaml_arg,
            log_map_path,
            start_world_cmd,
            map_server_node,
            nav2_bringup_cmd,
            waypoint_manager_node,
            gui_node,
            rviz_node,
            auto_init_node,
        ]
    )
