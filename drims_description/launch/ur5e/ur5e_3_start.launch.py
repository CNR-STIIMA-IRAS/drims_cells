from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition

def generate_launch_description():

    inner_launch = PathJoinSubstitution([
        FindPackageShare("drims_description"),
        "launch",
        "ur5e",
        "drims_ur5e_factory.launch.py"
    ])
    # camera_calibration_launch = PathJoinSubstitution([
    #     FindPackageShare("drims_description"),
    #     "launch",
    #     "ur5e",
    #     "camera_calibration_cell3.launch.py"
    # ])
    return LaunchDescription([

        DeclareLaunchArgument(
            "fake",
            default_value="true",
            description="Use fake hardware"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(inner_launch),
            launch_arguments={
                "fake": LaunchConfiguration("fake"),
                "robot_ip": "192.168.254.103",
                "cell_configuration_file": PathJoinSubstitution([
                    FindPackageShare("drims_description"),
                    "config",
                    "ur5e",
                    "configuration_cell_3.yaml"
                ])
            }.items()
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(camera_calibration_launch),
        #     launch_arguments={},
        #     condition=UnlessCondition(LaunchConfiguration('fake'))
        # )
    ])