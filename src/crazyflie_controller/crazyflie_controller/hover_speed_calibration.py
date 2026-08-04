#!/usr/bin/env python3
"""Empirically calibrates `hover_speed` (see DroneController.set_speeds in
controller_server.py) -- the constant z-velocity offset that a commanded
VelocityWorld of z=0 needs in order to actually mean "hold altitude" instead
of slowly falling (or climbing, if over-compensated).

Method: command a few different constant z-velocities in turn (x=y=0), hold
each for a few seconds, and measure the drone's *actual* average vertical
velocity from mocap over that window (linear regression on logged height,
after discarding an initial settling period). Fit a line through
(commanded z-velocity, measured z-velocity) across all candidates -- the
commanded value where that line crosses zero measured velocity is the
correct hover_speed.

IMPORTANT: run this against a drone that's already flying/hovering (e.g. via
the normal launch stack), but make sure that drone's controller_server
process is stopped first -- both would publish to the same
cmd_velocity_world topic and fight each other. Nothing holds the drone up
once this script exits, so land it or relaunch controller_server (with the
new hover_speed) immediately after.

Usage:
    python3 hover_speed_calibration.py --drone_name cf09
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from crazyflie_interfaces.msg import VelocityWorld

RATE_HZ = 50.0


class HoverSpeedCalibration(Node):
    def __init__(self, drone_name, candidates, hold_duration, settle_duration):
        super().__init__('hover_speed_calibration')
        self.drone_name = drone_name
        self.candidates = candidates
        self.hold_duration = hold_duration
        self.settle_duration = settle_duration

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.vel_pub = self.create_publisher(VelocityWorld, f'{drone_name}/cmd_velocity_world', 10)

        self.candidate_idx = 0
        self.phase_start = None
        self.samples = []       # (t, z) for the candidate currently under test
        self.results = []       # (commanded_vz, measured_vz) once each candidate finishes
        self.done = False

        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)

    def _tick(self):
        if self.done:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.phase_start is None:
            self.phase_start = now
            self.samples = []
            vz = self.candidates[self.candidate_idx]
            self.get_logger().info(f'testing commanded z-velocity = {vz:.3f} ...')

        vz = self.candidates[self.candidate_idx]
        msg = VelocityWorld()
        msg.vel.x = 0.0
        msg.vel.y = 0.0
        msg.vel.z = float(vz)
        msg.yaw_rate = 0.0
        self.vel_pub.publish(msg)

        try:
            trans = self.tf_buffer.lookup_transform('mocap', self.drone_name, rclpy.time.Time())
            self.samples.append((now - self.phase_start, trans.transform.translation.z))
        except tf2_ros.TransformException:
            pass

        if now - self.phase_start >= self.hold_duration:
            self._finish_candidate(vz)

    def _finish_candidate(self, vz):
        samples = np.array(self.samples)
        settled = samples[samples[:, 0] >= self.settle_duration]
        if len(settled) < 2:
            self.get_logger().error('not enough samples after settling window -- increase hold_duration')
            self.done = True
            return

        # z = slope*t + intercept -- slope is the measured vertical velocity
        slope, _ = np.polyfit(settled[:, 0], settled[:, 1], 1)
        self.get_logger().info(f'  measured actual vertical velocity = {slope:.4f} m/s')
        self.results.append((vz, slope))

        self.candidate_idx += 1
        self.phase_start = None

        if self.candidate_idx >= len(self.candidates):
            self._report()
            self.done = True

    def _report(self):
        commanded = np.array([r[0] for r in self.results])
        measured = np.array([r[1] for r in self.results])
        # fit measured = a*commanded + b, solve for commanded where measured == 0
        a, b = np.polyfit(commanded, measured, 1)

        self.get_logger().info('')
        self.get_logger().info(f'commanded vs measured: {list(zip(commanded.tolist(), measured.tolist()))}')
        self.get_logger().info(f'fit: measured = {a:.4f} * commanded + {b:.4f}')

        if abs(a) < 1e-6:
            self.get_logger().error('fit slope too close to zero -- try more spread-out candidate values')
            return

        calibrated_hover_speed = -b / a
        self.get_logger().info(f'==> calibrated hover_speed = {calibrated_hover_speed:.4f}')
        self.get_logger().info('Set this as hover_speed_sim in sim_drone.launch.py (or hover_speed_real for real flight).')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone_name', default='cf09')
    parser.add_argument('--candidates', type=float, nargs='+', default=[0.0, 0.15, 0.30])
    parser.add_argument('--hold_duration', type=float, default=6.0)
    parser.add_argument('--settle_duration', type=float, default=1.5)
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = HoverSpeedCalibration(parsed.drone_name, parsed.candidates, parsed.hold_duration, parsed.settle_duration)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
