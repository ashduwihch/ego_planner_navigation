#!/usr/bin/env python3
"""Take off and settle in PX4 local NED before starting planner navigation."""

import argparse
import math
import time

import rclpy
from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from controller.msg import ControlDebug, DesiredState


class Takeoff(Node):
    def __init__(self, altitude, velocity_tolerance, require_ev_fusion=False,
                 ev_ready_seconds=1.0, use_sim_time=True):
        super().__init__('controller_takeoff', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=use_sim_time)])
        self.altitude = altitude
        self.velocity_tolerance = velocity_tolerance
        self.require_ev_fusion = require_ev_fusion
        self.ev_ready_seconds = ev_ready_seconds
        self.local_position = None
        self.position_received_at = None
        self.takeoff_reference = None
        self.status = 'waiting_for_controller'
        self.stable_since = None
        self.ev_fusion_active = False
        self.ev_flags_received_at = None
        self.ev_fusion_ready_since = None
        self.controller_debug = None
        self.publisher = self.create_publisher(DesiredState, '/controller/desired_state', 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, '/px4ctrl/status', self.status_callback, 10)
        self.create_subscription(ControlDebug, '/px4ctrl/debug', self.debug_callback, 10)
        self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
            self.estimator_status_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.05, self.publish_reference)

    def position_callback(self, msg):
        values = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading)
        if (msg.timestamp and msg.xy_valid and msg.z_valid and msg.v_xy_valid and
                msg.v_z_valid and not msg.dead_reckoning and
                all(math.isfinite(value) for value in values)):
            self.local_position = msg
            self.position_received_at = self.get_clock().now()
        else:
            self.local_position = None
            self.stable_since = None

    def status_callback(self, msg):
        self.status = msg.data

    def debug_callback(self, msg):
        self.controller_debug = msg

    def estimator_status_callback(self, msg):
        now = self.get_clock().now()
        if self.ev_flags_received_at is not None and not (
                0 <= (now - self.ev_flags_received_at).nanoseconds <= 2_000_000_000):
            self.ev_fusion_ready_since = None
        self.ev_flags_received_at = now
        self.ev_fusion_active = msg.cs_ev_pos and msg.cs_ev_hgt and msg.cs_ev_vel
        if self.ev_fusion_active:
            if self.ev_fusion_ready_since is None:
                self.ev_fusion_ready_since = self.ev_flags_received_at
        else:
            self.ev_fusion_ready_since = None

    def ev_fusion_ready(self):
        if not self.require_ev_fusion:
            return True
        if not self.ev_fusion_active or self.ev_flags_received_at is None or \
                self.ev_fusion_ready_since is None:
            return False
        now = self.get_clock().now()
        return (0 <= (now - self.ev_flags_received_at).nanoseconds <= 2_000_000_000 and
                (now - self.ev_fusion_ready_since).nanoseconds >= self.ev_ready_seconds * 1e9)

    def publish_reference(self):
        p = self.local_position
        reference = self.takeoff_reference
        now = self.get_clock().now()
        if p is None or not 0 <= (now - self.position_received_at).nanoseconds <= 500_000_000:
            self.stable_since = None
            return
        if reference is None:
            if not self.ev_fusion_ready():
                self.status = 'waiting_for_external_vision_fusion'
                return
            self.takeoff_reference = (p.x, p.y, p.z - self.altitude, p.heading)
            reference = self.takeoff_reference
        msg = DesiredState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'px4_local_ned'
        msg.position.x, msg.position.y, msg.position.z, msg.yaw = reference
        self.publisher.publish(msg)

        altitude_reached = p.z <= reference[2] + 0.08
        speed = math.sqrt(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz)
        controller_tracking = self.status == 'state_tracking' or self.status.startswith('tracking_')
        if altitude_reached and speed <= self.velocity_tolerance and controller_tracking:
            if self.stable_since is None:
                self.stable_since = time.monotonic()
        else:
            self.stable_since = None


def main(args=None):
    parser = argparse.ArgumentParser(description='Take off before sending an EGO planner goal')
    parser.add_argument('--altitude', type=float, default=0.5,
                        help='climb distance in metres (default: 0.5)')
    parser.add_argument('--velocity-tolerance', type=float, default=0.15,
                        help='settling speed limit in m/s (default: 0.15)')
    parser.add_argument('--wait-timeout', type=float, default=45.0,
                        help='wall-clock timeout in seconds (default: 45)')
    parser.add_argument('--require-ev-fusion', action='store_true',
                        help='wait for PX4 external-vision position, height, and velocity fusion')
    parser.add_argument('--ev-ready-seconds', type=float, default=1.0,
                        help='required continuous EV fusion time before takeoff')
    parser.add_argument('--use-sim-time', choices=('true', 'false'), default='true')
    options = parser.parse_args(args)
    if (not all(math.isfinite(value) for value in
                (options.altitude, options.velocity_tolerance, options.wait_timeout,
                 options.ev_ready_seconds)) or
            options.altitude <= 0.0 or options.velocity_tolerance <= 0.0 or
            options.wait_timeout <= 0.0 or options.ev_ready_seconds < 0.0):
        parser.error('altitude, velocity tolerance, timeout, and EV ready time must be finite and valid')

    rclpy.init()
    node = Takeoff(options.altitude, options.velocity_tolerance,
                   options.require_ev_fusion, options.ev_ready_seconds,
                   options.use_sim_time == 'true')
    deadline = time.monotonic() + options.wait_timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.stable_since is not None and time.monotonic() - node.stable_since >= 0.8:
                print(f'PX4Ctrl takeoff complete: climbed {options.altitude:g} m and settled.')
                return 0
        p = node.local_position
        position_report = ('unavailable' if p is None else
                           f'z={p.z:.3f} m, vz={p.vz:.3f} m/s')
        debug = node.controller_debug
        debug_report = ('unavailable' if debug is None else
                        f'state={debug.state}, armed={debug.armed}, offboard={debug.offboard}, '
                        f'local_valid={debug.local_valid}, failsafe={debug.failsafe}, '
                        f'thrust={debug.thrust:.3f}, tilt={debug.tilt_deg:.1f} deg, '
                        f'command_age={debug.command_age_sec:.3f} s')
        raise RuntimeError(
            f'takeoff did not settle before timeout; last status: {node.status}; '
            f'EV fusion active: {node.ev_fusion_active}; PX4 position: {position_report}; '
            f'controller debug: {debug_report}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f'takeoff: {exc}')
        raise SystemExit(1)
