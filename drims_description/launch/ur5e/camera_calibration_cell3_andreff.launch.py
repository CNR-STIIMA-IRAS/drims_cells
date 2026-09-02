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
                "0.622161",
                "--y",
                "0.419075",
                "--z",
                "0.622017",
                "--qx",
                "-0.702917",
                "--qy",
                "-5.4977e-05",
                "--qz",
                "0.711099",
                "--qw",
                "0.0156858",
                # "--roll",
                # "2.06414",
                # "--pitch",
                # "-1.59575",
                # "--yaw",
                # "-1.09971",
            ],
        ),
    ]
    return LaunchDescription(nodes)
