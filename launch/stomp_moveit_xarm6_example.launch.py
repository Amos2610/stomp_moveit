import os
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
    # パス解決
    package_path = FindPackageShare("xarm_moveit_config").perform(context)
    stomp_param_path = os.path.join(package_path, "config", "xarm6", "stomp_planning.yaml")

    # 読み込み
    with open(stomp_param_path, "r") as f:
        stomp_param_raw = yaml.safe_load(f)

    print("✅ stomp_param contents:")
    print(stomp_param_raw)

    return [
        Node(
            package="xarm_joint_planner_cpp",
            executable="joint_planner_node",
            name="xarm6_joint_planner",
            output="screen",
            parameters=[
                {
                    "robot_description": Command([
                        "xacro ", PathJoinSubstitution([
                            FindPackageShare("xarm_description"),
                            "urdf", "xarm_device.urdf.xacro"
                        ]),
                        " ", "dof:=6",
                        " ", "robot_type:=xarm",
                        " ", "add_gripper:=true"
                    ])
                },
                {
                    "robot_description_semantic": Command([
                        "xacro ", PathJoinSubstitution([
                            FindPackageShare("xarm_moveit_config"),
                            "srdf", "xarm.srdf.xacro"
                        ]),
                        " ", "dof:=6",
                        " ", "robot_type:=xarm",
                        " ", "add_gripper:=true"
                    ])
                },
                {'default_planning_pipeline': 'stomp'},
                stomp_param_raw,
                {'planning_pipelines': ['stomp']},
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
