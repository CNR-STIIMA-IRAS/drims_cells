""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: table_top -> oak-d-base-frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "table_top",
                "--child-frame-id",
                "oak-d-base-frame",
                "--x",
                "0.742959",
                "--y",
                "0.483412",
                "--z",
                "0.610681",
                "--qx",
                "0.718417",
                "--qy",
                "-0.0169895",
                "--qz",
                "-0.695402",
                "--qw",
                "-0.00203008",
                # "--roll",
                # "0.680055",
                # "--pitch",
                # "-1.61302",
                # "--yaw",
                # "-2.44038",
            ],
        ),
    ]
    return LaunchDescription(nodes)
