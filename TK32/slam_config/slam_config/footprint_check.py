"""Inspect runtime Nav2 polygon-footprint wiring without moving the robot.

Usage after starting MUR:
    ros2 run slam_config check_dynamic_footprint

This program never publishes velocities or footprints. It only diagnoses
whether a dynamic-footprint *input* is present and whether Nav2 emits its
transformed *output*. It cannot establish that the polygon is correctly
computed for all joint states: confirm that separately with the arms parked.
"""

import time

import rclpy
from geometry_msgs.msg import Polygon, PolygonStamped
from rclpy.node import Node

INPUT_TOPIC = '/local_costmap/footprint'
OUTPUT_TOPIC = '/local_costmap/published_footprint'
GLOBAL_TOPIC = '/global_costmap/footprint'


class FootprintCheck(Node):
    def __init__(self):
        super().__init__('slam_config_footprint_check')
        self.input_count = 0
        self.output_count = 0
        self.last_input = None
        self.last_output = None
        self.input_subscription = self.create_subscription(
            Polygon, INPUT_TOPIC, self.on_input, 10)
        self.output_subscription = self.create_subscription(
            PolygonStamped, OUTPUT_TOPIC, self.on_output, 10)

    def on_input(self, msg):
        self.input_count += 1
        self.last_input = msg

    def on_output(self, msg):
        self.output_count += 1
        self.last_output = msg

    def print_report(self):
        topic_types = dict(self.get_topic_names_and_types())
        for topic in (INPUT_TOPIC, OUTPUT_TOPIC, GLOBAL_TOPIC):
            publishers = self.get_publishers_info_by_topic(topic)
            print(f'\n{topic}')
            print(f'  types: {topic_types.get(topic, []) or "(not advertised)"}')
            print(f'  publishers: {len(publishers)}')
            for info in publishers:
                print(f'    - {info.node_namespace}/{info.node_name}: {info.topic_type}')

        print('\nObserved messages in the 5-second sample:')
        print(f'  dynamic footprint inputs: {self.input_count}')
        print(f'  transformed footprint outputs: {self.output_count}')
        if self.last_input is not None:
            pts = self.last_input.points
            print(f'  most recent input polygon: {len(pts)} vertices')
            if pts:
                print('  input x range: '
                      f'{min(p.x for p in pts):.3f} to {max(p.x for p in pts):.3f} m')
                print('  input y range: '
                      f'{min(p.y for p in pts):.3f} to {max(p.y for p in pts):.3f} m')
        if self.last_output is not None:
            print(f'  latest transformed frame: {self.last_output.header.frame_id}')

        expected_types = topic_types.get(INPUT_TOPIC, [])
        input_publishers = self.get_publishers_info_by_topic(INPUT_TOPIC)
        if 'geometry_msgs/msg/Polygon' not in expected_types or not input_publishers:
            print('\nFAIL: No Polygon publisher found on /local_costmap/footprint.')
            print('  MPPI can only use the static fallback until mur_footprint')
            print('  publishes geometry_msgs/msg/Polygon to that input topic.')
            print('  published_footprint is an OUTPUT, not the right input.')
            return 1
        print('\nPASS: A correctly typed dynamic-footprint INPUT publisher exists.')
        if not self.input_count:
            print('  No input message received during this window; ensure the')
            print('  publisher is active and check its QoS and update rate.')
        if not self.output_count:
            print('  No Nav2 transformed footprint output received; check Nav2.')
        if not self.get_publishers_info_by_topic(GLOBAL_TOPIC):
            print('  NOTE: Global costmap has no dynamic-footprint input publisher.')
            print('  Its planner may still use the configured static footprint.')
        print('  IMPORTANT: verify the polygon changes correctly with arm pose')
        print('  and is expressed in the local costmap robot base frame.')
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = FootprintCheck()
    try:
        until = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = node.print_report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
