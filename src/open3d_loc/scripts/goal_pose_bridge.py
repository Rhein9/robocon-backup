#!/usr/bin/env python3

import functools

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__('goal_pose_bridge')

        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('action_name', 'navigate_to_pose')

        self._goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self._action_name = self.get_parameter('action_name').value
        self._action_client = ActionClient(self, NavigateToPose, self._action_name)
        self._pending_futures = set()
        self._goal_sequence = 0

        self._goal_pose_subscription = self.create_subscription(
            PoseStamped,
            self._goal_pose_topic,
            self._goal_pose_callback,
            10,
        )

        self.get_logger().info(
            f'Listening for RViz goals on {self._goal_pose_topic} and forwarding them to '
            f'{self._action_name}.'
        )

    def _track_future(self, future) -> None:
        self._pending_futures.add(future)
        future.add_done_callback(lambda done_future: self._pending_futures.discard(done_future))

    def _goal_pose_callback(self, pose_msg: PoseStamped) -> None:
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f'Nav2 action server {self._action_name} is not available yet. '
                'Start nav2 first, then send the goal again.'
            )
            return

        self._goal_sequence += 1
        request_id = self._goal_sequence

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        pose = pose_msg.pose.position
        self.get_logger().info(
            f'Sending nav2 goal #{request_id} in frame {pose_msg.header.frame_id or "map"} '
            f'-> x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}'
        )

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=functools.partial(self._feedback_callback, request_id),
        )
        self._track_future(future)
        future.add_done_callback(functools.partial(self._goal_response_callback, request_id))

    def _feedback_callback(self, request_id: int, feedback_msg) -> None:
        current_pose = feedback_msg.feedback.current_pose.pose.position
        self.get_logger().debug(
            f'Goal #{request_id} feedback -> x={current_pose.x:.3f}, '
            f'y={current_pose.y:.3f}, z={current_pose.z:.3f}'
        )

    def _goal_response_callback(self, request_id: int, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send goal #{request_id}: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f'Nav2 rejected goal #{request_id}.')
            return

        self.get_logger().info(f'Nav2 accepted goal #{request_id}.')
        result_future = goal_handle.get_result_async()
        self._track_future(result_future)
        result_future.add_done_callback(functools.partial(self._result_callback, request_id))

    def _result_callback(self, request_id: int, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to get result for goal #{request_id}: {exc}')
            return

        status_text = {
            GoalStatus.STATUS_UNKNOWN: 'unknown',
            GoalStatus.STATUS_ACCEPTED: 'accepted',
            GoalStatus.STATUS_EXECUTING: 'executing',
            GoalStatus.STATUS_CANCELING: 'canceling',
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }.get(result.status, f'status={result.status}')

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Nav2 goal #{request_id} succeeded.')
        else:
            self.get_logger().warn(f'Nav2 goal #{request_id} finished with {status_text}.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
