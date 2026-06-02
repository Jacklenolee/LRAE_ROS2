#!/usr/bin/env python3
import argparse
import math
import sys

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class RangerEntitySpawner(Node):
    def __init__(self, args):
        super().__init__('spawn_ranger_entity')
        self.args = args
        self.robot_description = ''
        self.entity_seen = False

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, args.topic, self._robot_description_cb, latched_qos)
        self.create_subscription(ModelStates, '/model_states', self._model_states_cb, 10)

    def _robot_description_cb(self, msg):
        self.robot_description = msg.data

    def _model_states_cb(self, msg):
        self.entity_seen = self.args.entity in msg.name

    def wait_for_robot_description(self):
        deadline = self.get_clock().now() + Duration(seconds=self.args.timeout)
        while rclpy.ok() and not self.robot_description:
            if self.get_clock().now() > deadline:
                self.get_logger().error(f'Timed out waiting for {self.args.topic}')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def wait_for_entity(self):
        deadline = self.get_clock().now() + Duration(seconds=self.args.entity_timeout)
        while rclpy.ok() and not self.entity_seen:
            if self.get_clock().now() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def spawn(self):
        if not self.wait_for_robot_description():
            return 1

        client = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_logger().info(f'Waiting for /spawn_entity, timeout = {self.args.timeout:.0f}')
        if not client.wait_for_service(timeout_sec=self.args.timeout):
            self.get_logger().error('Service /spawn_entity unavailable')
            return 1

        request = SpawnEntity.Request()
        request.name = self.args.entity
        request.xml = self.robot_description
        request.initial_pose = self._make_pose()

        self.get_logger().info(f'Calling /spawn_entity for {self.args.entity}')
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            self.get_logger().error('Spawn service call was interrupted')
            return 1

        response = future.result()
        status = response.status_message
        if response.success:
            self.get_logger().info(f'Spawn status: {status}')
            return 0

        if 'pushed to spawn queue' in status:
            self.get_logger().warn(f'Gazebo queued entity slowly: {status}')
            if self.wait_for_entity():
                self.get_logger().info(f'Entity {self.args.entity} appeared in /model_states')
            else:
                self.get_logger().warn(
                    f'Could not confirm {self.args.entity} on /model_states; continuing because Gazebo accepted the queue'
                )
            return 0

        self.get_logger().error(f'Spawn failed: {status}')
        return 1

    def _make_pose(self):
        pose = Pose()
        pose.position.x = self.args.x
        pose.position.y = self.args.y
        pose.position.z = self.args.z
        half_yaw = self.args.yaw * 0.5
        pose.orientation.w = math.cos(half_yaw)
        pose.orientation.z = math.sin(half_yaw)
        return pose


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--entity', required=True)
    parser.add_argument('--topic', default='robot_description')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--entity-timeout', type=float, default=60.0)
    return parser.parse_args(argv)


def main(argv=None):
    rclpy.init(args=argv)
    cli_args = sys.argv if argv is None else argv
    args_without_ros = rclpy.utilities.remove_ros_args(args=cli_args)[1:]
    node = RangerEntitySpawner(parse_args(args_without_ros))
    try:
        return node.spawn()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
