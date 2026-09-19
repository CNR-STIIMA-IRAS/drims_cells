#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action import GoalResponse, CancelResponse

from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GripperBridge(Node):
    def __init__(self):
        super().__init__('tiago_gripper_bridge_node')

        # Declare parameter for selecting gripper side ('right' or 'left', default: 'right')
        self.declare_parameter('gripper_side', 'right')

        # Action server exposed to clients
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_action_controller/gripper_cmd',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Action clients for BOTH grippers
        self._right_gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_right_controller/follow_joint_trajectory'
        )

        self._left_gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_left_controller/follow_joint_trajectory'
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal: {goal_request.command.position:.3f}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Select target gripper based on parameter
        gripper_side = self.get_parameter('gripper_side').get_parameter_value().string_value.lower()
        if gripper_side == 'left':
            target_client = self._left_gripper_client
            joint_name = 'gripper_left_finger_joint'
            side_name = 'LEFT'
        else:
            target_client = self._right_gripper_client
            joint_name = 'gripper_right_finger_joint'
            side_name = 'RIGHT'

        self.get_logger().info(f'Selected {side_name} gripper controller for command.')

        # Wait for the remote action server to be available
        if not target_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'{side_name} Gripper action server not available!')
            goal_handle.abort()
            return GripperCommand.Result(position=0.0, effort=0.0, stalled=False, reached_goal=False)

        # Convert GripperCommand to FollowJointTrajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [goal_handle.request.command.position]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        goal_msg.trajectory.points = [point]

        # Send goal to the underlying FollowJointTrajectory action server
        self.get_logger().info(f'Sending {side_name} gripper command to FollowJointTrajectory: {point.positions[0]:.3f}')
        send_goal_future = target_client.send_goal_async(goal_msg)
        goal_response = await send_goal_future

        if not goal_response.accepted:
            self.get_logger().error(f'Goal rejected by {side_name} FollowJointTrajectory')
            goal_handle.abort()
            return GripperCommand.Result(position=0.0, effort=0.0, stalled=False, reached_goal=False)

        # Wait for result
        result_future = goal_response.get_result_async()
        result = await result_future

        # If goal aborted or failed because gripper stalled against object, report succeeded with stalled=True
        is_stalled = (result.status != 4)  # GoalStatus.STATUS_SUCCEEDED is 4
        if is_stalled:
            self.get_logger().info(f'{side_name} Gripper contact detected / stalled on object (action status={result.status}). Succeeding GripperCommand.')

        self.get_logger().info(f'{side_name} Gripper goal finished successfully.')
        goal_handle.succeed()

        return GripperCommand.Result(
            position=goal_handle.request.command.position,
            effort=goal_handle.request.command.max_effort,
            stalled=is_stalled,
            reached_goal=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
