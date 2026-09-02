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
                "0.71918",
                "--y",
                "0.456763",
                "--z",
                "0.611237",
                "--qx",
                "-0.694337",
                "--qy",
                "-0.0213296",
                "--qz",
                "0.719333",
                "--qw",
                "-0.00139696",
                # "--roll",
                # "0.751981",
                # "--pitch",
                # "-1.52302",
                # "--yaw",
                # "-2.41781",
            ],
        ),
    ]
    return LaunchDescription(nodes)
