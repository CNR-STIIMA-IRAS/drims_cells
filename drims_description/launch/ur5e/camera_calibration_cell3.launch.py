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
                "0.62707",
                "--y",
                "0.428523",
                "--z",
                "0.636255",
                "--qx",
                "-0.702164",
                "--qy",
                "-0.00658672",
                "--qz",
                "0.711981",
                "--qw",
                "0.00236064",
                # "--roll",
                # "0.412838",
                # "--pitch",
                # "-1.55568",
                # "--yaw",
                # "-2.74141",
            ],
        ),
    ]
    return LaunchDescription(nodes)
