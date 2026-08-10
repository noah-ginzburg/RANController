"""Entry point: wires the ROS model to the Qt window and runs the loop."""

import sys
import time

import rclpy
from PyQt5.QtWidgets import QApplication

from crazyflie_debug_gui.telemetry_model import TelemetryModel
from crazyflie_debug_gui.window import DroneWindow


REFRESH_HZ = 10.0


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryModel()          # the ROS node: subscriptions + derived values

    app = QApplication(sys.argv)
    window = DroneWindow(node)       # window is wrapped around that node
    window.show()

    # Qt drives the loop instead of app.exec_() so that SIGINT (Ctrl-C, and the
    # SIGINT ros2 launch sends on shutdown) is actually delivered to Python.
    last_refresh = 0.0
    try:
        while rclpy.ok() and window.isVisible():
            rclpy.spin_once(node, timeout_sec=0.005)  # idle without busy-waiting
            for _ in range(10):                        # then drain what's ready
                rclpy.spin_once(node, timeout_sec=0)

            now = time.monotonic()
            if now - last_refresh >= 1.0 / REFRESH_HZ:
                window.refresh()
                last_refresh = now

            app.processEvents()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
