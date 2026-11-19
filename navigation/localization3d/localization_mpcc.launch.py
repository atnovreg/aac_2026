"""Feed a fixed-map pose and track progress to MPCC from a MID-360 stream."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    repo = Path(__file__).resolve().parents[2]
    config = LaunchConfiguration("fastlio_config")
    clock = LaunchConfiguration("use_sim_time")
    ros_clock = {"use_sim_time": ParameterValue(clock, value_type=bool)}

    def python_node(module, arguments):
        return ExecuteProcess(
            cmd=[sys.executable, "-m", module, "--ros-args",
                 "-p", ["use_sim_time:=", clock], *arguments],
            cwd=str(repo), output="screen", on_exit=Shutdown(),
        )

    def localizer(context):
        seed = LaunchConfiguration("initial_base_pose").perform(context)
        initial = "global_initialize:=true" if seed == "auto" else f"initial_base_pose:={seed}"
        return [python_node("navigation.localization3d.ros_localizer", [
            "-p", ["map_dir:=", LaunchConfiguration("map_dir")],
            "-p", initial,
            "-p", ["fastlio_config:=", config],
        ])]

    return LaunchDescription([
        DeclareLaunchArgument("map_dir", description="Directory containing map.pcd and map_evidence.npz"),
        DeclareLaunchArgument("track_csv", description="MPCC track.csv in the fixed map frame"),
        DeclareLaunchArgument("initial_base_pose", description="auto or a map pose [x,y,z,roll,pitch,yaw], m and radians"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fastlio_config", default_value=str(repo / "navigation/fastlio/config/mid360_bag.yaml")),
        SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
        Node(package="bag_adapter", executable="pointcloud_to_livox",
             parameters=[ros_clock], output="screen", on_exit=Shutdown()),
        Node(package="fast_lio", executable="fastlio_mapping",
             parameters=[config, ros_clock, {"publish.scan_publish_en": True,
                                              "publish.scan_bodyframe_pub_en": True,
                                              "pcd_save.pcd_save_en": False}],
             output="screen", on_exit=Shutdown()),
        OpaqueFunction(function=localizer),
        python_node("control.acados_mpcc.ros_input", [
            "-p", ["track_csv:=", LaunchConfiguration("track_csv")],
        ]),
    ])
