import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess
from launch.actions import OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_processes(context):
    model_dir = Path(__file__).resolve().parent / "models"
    resources = str(model_dir)
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resources += os.pathsep + os.environ["GZ_SIM_RESOURCE_PATH"]
    command = ["gz", "sim", "-r"]
    if LaunchConfiguration("gui").perform(context) == "false":
        command.append("-s")
    command.append(str(model_dir / "world.sdf"))
    gazebo = ExecuteProcess(cmd=command, name="gazebo", output="screen")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="motion_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        output="screen",
    )
    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resources),
        RegisterEventHandler(OnProcessExit(
            target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason="Gazebo exited"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="Bridge exited"))],
        )),
        gazebo,
        bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=start_processes),
    ])