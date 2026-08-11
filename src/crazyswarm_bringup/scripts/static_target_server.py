#!/usr/bin/env python3

"""
Publish static (non-flying) targets, so they look like drones to the rest of the stack.

crazyflie_sim only knows about robots it simulates, so rather than patch it,
this node fakes the one thing downstream actually reads: a TF frame.
spherical_RAN_server_lloyd finds its targets by looking up `mocap -> <name>`
(see _get_targets_from_tf), so a broadcast frame is indistinguishable from a
real drone as far as the RAN model is concerned.

Per enabled target in the yaml: a TF frame, a sphere + label marker, and (if
`draggable`) an interactive marker to pull around in rviz. The markers live in
the target's *own* frame, so a drag only moves the frame and they follow.
"""

import math
import os

from geometry_msgs.msg import Point, TransformStamped, Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)
import yaml

from interactive_markers import InteractiveMarkerServer


TF_RATE = 20.0
# Size of the drag handles, as a multiple of the target sphere. rviz scales the
# axis arrows it auto-generates off the interactive marker's `scale`, so this is
# the knob for "the arrows are too big/small".
HANDLE_SCALE = 2.0
# rviz's InteractiveMarkers display has to be pointed at this namespace.
IM_NAMESPACE = 'static_targets'


class StaticTarget:
    """One target: where it is, how it draws, how strongly the RAN model sees it."""

    def __init__(self, name, cfg):
        self.name = name
        self.position = [float(p) for p in cfg['position']]
        self.quality = float(cfg.get('quality', 20.0))
        self.draggable = bool(cfg.get('draggable', False))
        self.color = [float(c) for c in cfg.get('color', [1.0, 0.35, 0.1, 0.9])]
        self.scale = float(cfg.get('scale', 0.12))


def load_targets(path):
    """Parse the yaml into (frame, [StaticTarget]); disabled entries are dropped."""
    with open(path, 'r') as f:
        content = yaml.safe_load(f)
    targets = [StaticTarget(str(name), cfg)
               for name, cfg in content['static_targets'].items()
               if cfg.get('enabled', True)]
    return content.get('frame', 'mocap'), targets


class StaticTargetServer(Node):

    def __init__(self):
        super().__init__('static_target_server')

        self.declare_parameter('targets_yaml', '')
        # Replaces the yaml's per-target `quality`, in file order. Only the
        # marker labels use it here -- the RAN server gets qualities as its own
        # parameter -- but the two have to agree.
        self.declare_parameter('quality_overrides', Parameter.Type.DOUBLE_ARRAY)

        path = self.get_parameter('targets_yaml').value
        if not os.path.isfile(path):
            raise RuntimeError(f'targets_yaml missing or not found: {path!r}')

        self.frame, self.targets = load_targets(path)

        # get_parameter_or, because a type-only declaration (the only kind that
        # can express an empty list) stays uninitialized when nothing passes it.
        qualities = self.get_parameter_or('quality_overrides').value or []
        if qualities:
            if len(qualities) != len(self.targets):
                raise RuntimeError(f'quality_overrides has {len(qualities)} values but '
                                   f'{len(self.targets)} targets are enabled')
            for t, q in zip(self.targets, qualities):
                t.quality = q

        for t in self.targets:
            self.get_logger().info(f'static target {t.name}: pos={t.position} '
                                   f'quality={t.quality} draggable={t.draggable}')

        self.tfbr = TransformBroadcaster(self)
        self.create_timer(1.0 / TF_RATE, self._publish_tf)
        self._publish_tf()

        # Latched: the markers never change (they ride their TF frame), so
        # publishing once is enough, and a late-starting rviz still gets them.
        self.marker_pub = self.create_publisher(
            MarkerArray, '~/markers',
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.marker_pub.publish(self._markers())

        self.im_server = InteractiveMarkerServer(self, IM_NAMESPACE)
        for t in self.targets:
            if t.draggable:
                self.im_server.insert(self._interactive_marker(t),
                                      feedback_callback=self._on_feedback)
        self.im_server.applyChanges()

    def _publish_tf(self):
        """Broadcast one <frame> -> <target> transform each; this is what RAN reads."""
        stamp = self.get_clock().now().to_msg()
        msgs = []
        for t in self.targets:
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame
            msg.child_frame_id = t.name
            msg.transform.translation = Vector3(x=t.position[0], y=t.position[1],
                                                z=t.position[2])
            msg.transform.rotation.w = 1.0
            msgs.append(msg)
        self.tfbr.sendTransform(msgs)

    def _markers(self):
        """A sphere and a label per target, anchored in the target's own frame.

        frame_locked is what makes a drag move them: without it rviz resolves
        the frame once, when the marker arrives, and never re-transforms -- so
        the frame moves, the drag shell moves with it, and the sphere stays put.
        """
        arr = MarkerArray()
        for i, t in enumerate(self.targets):
            sphere = Marker()
            sphere.header.frame_id = t.name
            sphere.frame_locked = True
            sphere.ns = 'static_targets'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.pose.orientation.w = 1.0
            sphere.scale = Vector3(x=t.scale, y=t.scale, z=t.scale)
            sphere.color = ColorRGBA(r=t.color[0], g=t.color[1], b=t.color[2], a=t.color[3])

            label = Marker()
            label.header.frame_id = t.name
            label.frame_locked = True
            label.ns = 'static_target_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.z = t.scale
            label.pose.orientation.w = 1.0
            label.scale.z = 0.08
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            label.text = f'{t.name} (q={t.quality:g})'

            arr.markers += [sphere, label]
        return arr

    def _interactive_marker(self, target):
        """A grab-anywhere shell for rough placement, plus three axis arrows for fine."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame
        int_marker.name = target.name
        int_marker.description = target.name
        int_marker.scale = HANDLE_SCALE * target.scale
        int_marker.pose.position = Point(x=target.position[0], y=target.position[1],
                                         z=target.position[2])
        int_marker.pose.orientation.w = 1.0

        shell = Marker()
        shell.type = Marker.SPHERE
        shell.scale = Vector3(x=1.6 * target.scale, y=1.6 * target.scale, z=1.6 * target.scale)
        shell.color = ColorRGBA(r=target.color[0], g=target.color[1], b=target.color[2], a=0.35)
        shell.pose.orientation.w = 1.0

        grab = InteractiveMarkerControl()
        grab.interaction_mode = InteractiveMarkerControl.MOVE_3D
        grab.always_visible = True
        grab.markers.append(shell)
        int_marker.controls.append(grab)

        # A control's orientation is the axis it slides along, so x is identity
        # and y/z are 90deg rotations about z and y. rviz draws the arrows.
        half = math.sqrt(0.5)
        for w, x, y, z in ((half, half, 0.0, 0.0), (half, 0.0, 0.0, half), (half, 0.0, half, 0.0)):
            control = InteractiveMarkerControl()
            control.orientation.w, control.orientation.x = w, x
            control.orientation.y, control.orientation.z = y, z
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)

        return int_marker

    def _on_feedback(self, feedback):
        """Move the target's frame to wherever rviz dragged the handle."""
        target = next(t for t in self.targets if t.name == feedback.marker_name)
        p = feedback.pose.position
        target.position = [p.x, p.y, p.z]
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # In yaml form, so it can be pasted straight back into the config.
            self.get_logger().info(
                f'{target.name}: position: [{p.x:.6f}, {p.y:.6f}, {p.z:.6f}]')


def main():
    rclpy.init()
    node = StaticTargetServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C arrives as either, given the launch's 2s sigterm timeout
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
