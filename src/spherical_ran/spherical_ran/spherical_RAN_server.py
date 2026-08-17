import os
import time

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import ColorRGBA

import pyvista as pv
import tf2_ros
from rclpy.duration import Duration as RCLDuration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray


# Everything that used to be a module constant here now comes from
# config/ran_params.yaml instead, via the declare_parameter block below. The
# colormap is the one exception and stays in code, because a nested tuple of
# rgb triples doesn't map onto any of the ROS parameter types cleanly.
RAN_VIS_COLORMAP = (    #blue -> cyan -> yellow -> red
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 1.0),
    (1.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
)
X = MAG = 0
Y = PHI = 1     #phi = xy angle
Z = THETA = 2   #theta = z axis angle
QUALITY = 3


class SphericalRANServer(Node):
    """Spherical attractor network over a plain icosphere.

    The node positions come from pyvista's Icosphere, and the connectivity
    matrix M is precomputed by generate_kernel_cache.py. That script reads n_sub
    and v from the same ran_params.yaml this node reads, so the two can't drift
    apart on their own. If they ever do end up mismatched, the cache check below
    refuses to start rather than run the model on a kernel that was built for a
    different mesh.
    """

    def __init__(self):
        super().__init__('spherical_RAN_server')
        self.declare_parameter('drone_name', 'cf01')
        self.declare_parameter('ran_vis', True)
        # Set by real_drone_teleop.launch.py. Under teleop the person at the
        # keyboard owns the drone, so the model must not be commanding a heading
        # at the same time; the controller would act on both and end up fighting
        # the keyboard. The model still runs and still draws in RViz, and only
        # the heading publish is held back.
        self.declare_parameter('teleop_enabled', False)

        # These defaults follow mean_field_model_3d.ipynb. In practice they're
        # set in ran_params.yaml, which overrides whatever is written here.
        self.declare_parameter('beta', 1.5)
        self.declare_parameter('v', 0.3)
        self.declare_parameter('sigma', 1.0)
        self.declare_parameter('kappa', 10.0)
        self.declare_parameter('u', 35.0)
        self.declare_parameter('rate', 1.0)
        # How concentrated the activation has to be, on a scale of 0 to 1,
        # before we trust its direction enough to publish a heading. The
        # resultant-length comment in update() explains what this actually
        # measures.
        self.declare_parameter('bump_threshold', 0.3)
        self.declare_parameter('n_sub', 3)
        self.declare_parameter('all_drones', [''])
        self.declare_parameter('target_quality', 20.0)
        self.declare_parameter('target_names', [''])
        self.declare_parameter('target_qualities', [0.0])

        # The model's fixed integration step. This is not the measured
        # wall-clock dt, and that's deliberate; the yaml and update() both
        # explain why.
        self.declare_parameter('dt', 0.1)
        # How close another drone has to get, in metres, before it counts as too
        # close.
        self.declare_parameter('neighbour_radius', 0.3)
        # Off by default, because it hasn't been tested in flight and because it
        # fights the attractor model rather than cooperating with it. When it's
        # on, a neighbour inside neighbour_radius flips the horizontal heading so
        # the drone backs straight out. The block in update() does the work.
        self.declare_parameter('collision_avoidance', False)
        self.declare_parameter('update_rate', 50.0)
        # How long to wait before spinning, so that TF and crazyflie_server are
        # up before the first update() starts doing lookups.
        self.declare_parameter('startup_delay', 10.0)
        self.declare_parameter(
            'kernel_cache', 'src/spherical_ran/spherical_ran/kernel_cache.npz')
        self.declare_parameter('vis_radius', 0.1)
        self.declare_parameter('vis_point_size', 0.007)
        self.declare_parameter('vis_z_offset', 0.015)
        self.declare_parameter('vis_z_min', -1.0)
        self.declare_parameter('vis_z_max', 2.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.drone_name = self.get_parameter('drone_name').value
        self.ran_vis = self.get_parameter('ran_vis').value
        self.teleop_enabled = self.get_parameter('teleop_enabled').value
        self.beta = self.get_parameter('beta').value
        self.v = self.get_parameter('v').value
        self.sigma = self.get_parameter('sigma').value
        self.kappa = self.get_parameter('kappa').value
        self.u = self.get_parameter('u').value
        self.rate = self.get_parameter('rate').value
        self.bump_threshold = self.get_parameter('bump_threshold').value
        self.n_sub = self.get_parameter('n_sub').value
        self.target_quality = self.get_parameter('target_quality').value
        target_names = self.get_parameter('target_names').value
        target_qualities = self.get_parameter('target_qualities').value
        self.target_quality_overrides = dict(zip(target_names, target_qualities))

        self.dt_model = float(self.get_parameter('dt').value)
        self.neighbour_radius = float(self.get_parameter('neighbour_radius').value)
        self.collision_avoidance = self.get_parameter('collision_avoidance').value
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.vis_radius = float(self.get_parameter('vis_radius').value)
        self.vis_point_size = float(self.get_parameter('vis_point_size').value)
        self.vis_z_offset = float(self.get_parameter('vis_z_offset').value)
        self.vis_z_min = float(self.get_parameter('vis_z_min').value)
        self.vis_z_max = float(self.get_parameter('vis_z_max').value)

        all_drones = self.get_parameter('all_drones').value
        self.other_drones = [d for d in all_drones if d != self.drone_name]

        self.nodes = self._generate_nodes(self.n_sub)
        self.num_nodes = len(self.nodes)
        self.dt = 0.0
        self.z = np.zeros(self.num_nodes)

        cache_path = self.get_parameter('kernel_cache').value
        try:
            data = np.load(cache_path)
            if (data['n_sub'] != self.n_sub or data['v'] != self.v):
                raise ValueError('cached kernel parameters do not match current parameters. ensure that both this server and the kernel cache script used the same v value')

            self.nodes = data['nodes']
            self.alphas = data['alphas']
            self.M = data['M']
            self.get_logger().info(f'Loaded cached kernel from {cache_path}')

        except (FileNotFoundError, ValueError, OSError) as e:
            self.get_logger().fatal(
                f'kernel cache unusable ({e}). Run generate_kernel_cache.py and restart.')
            raise RuntimeError(f'kernel cache unusable: {e}') from e

        self.targets = []
        # This stays None until tf answers. It used to start at the origin,
        # which meant the whole visualization got drawn on the floor at (0,0,0)
        # for however long the drone's transform took to show up, since every tf
        # lookup below fails quietly in the meantime.
        self.drone_world_pos = None

        self.heading_pub = self.create_publisher(Vector3, f'{self.drone_name}/desired_heading', 10)
        # rviz's MarkerArray display for this topic (crazyswarm_bringup/rviz/3_targets.rviz)
        # is configured Reliability Policy: Reliable -- match it, or rviz silently drops
        # every message (QoS mismatch warning, no markers rendered).
        vis_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.vis_pub = self.create_publisher(MarkerArray, f'{self.drone_name}/ran_viz', vis_qos)
        self._vis_count = 0
        if self.collision_avoidance:
            self.get_logger().warn(
                f'collision_avoidance ENABLED for {self.drone_name}: a drone within '
                f'{self.neighbour_radius:.2f} m reverses the heading xy. This '
                'overrides the attractor model whenever it triggers.')
        if self.teleop_enabled:
            self.get_logger().warn(
                f'teleop_enabled: {self.drone_name}/desired_heading will NOT be '
                'published. The model still runs and still draws in RViz.')
        self.timer = self.create_timer((1.0/self.update_rate), self.update)
        self.heading_msg = Vector3()
        self.heading_msg.x = 0.0
        self.heading_msg.y = 0.0
        self.heading_msg.z = 0.0
        self.prev_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        # self.dt = (now - self.prev_time).nanoseconds * 1e-9
        self.dt = self.dt_model

        self.targets = self._get_targets_from_tf()
        # self.targets = [(1.0, 0.0, np.pi/2, 20.0)]

        try:
            self_trans = self.tf_buffer.lookup_transform('mocap', self.drone_name, rclpy.time.Time())
            t = self_trans.transform.translation
            self.drone_world_pos = np.array([t.x, t.y, t.z])
        except tf2_ros.TransformException:
            pass

        b = self._generate_sensory_input(self.nodes, self.targets, self.kappa)
        if self.targets:
            peak = self.nodes[np.argmax(b)]
            # self.get_logger().info(
            #     f'target phi={self.targets[0][PHI]:.3f} theta={self.targets[0][THETA]:.3f} | '
            #     f'b_peak phi={peak[PHI]:.3f} theta={peak[THETA]:.3f} (b={np.max(b):.3f})'
            # )
        noise = self._rand_link_func(self.z, self.sigma * np.sqrt(self.dt), 1.0/np.sqrt(self.num_nodes))

        self.z = (self.z
            + self.dt * self.rate * (-(self.z) + np.tanh(self.u * (self.M @ self.z) + b - self.beta) - np.tanh(-self.beta))
            + np.sqrt(self.rate) * noise)

        if self.targets:
            zpeak = self.nodes[np.argmax(self.z)]
            # self.get_logger().info(
            #     f'z_peak phi={zpeak[PHI]:.3f} theta={zpeak[THETA]:.3f} '
            #     f'(z={np.max(self.z):.3f}, sum_z={np.sum(self.z):.3f})'
            # )

        vec = self._find_vel_avg_3D(self.nodes, self.z)
        # `vec` is a weighted average of unit-length node positions, so its own
        # length is what directional statistics calls the mean resultant length,
        # R. When R is 1 every active node points the same way, which means a
        # sharp well-formed bump. When R is 0 the activations cancel each other
        # out, so the network is still flat and hasn't committed to a direction.
        # Using R this way gives us the actual concentration of the population
        # vector, rather than an arbitrary cutoff on the raw activation values.
        r = np.linalg.norm(vec)

        for targ in self.other_drones:
            try:
                relative_pos = self.tf_buffer.lookup_transform(f'{self.drone_name}', targ, rclpy.time.Time())
            except tf2_ros.TransformException:
                continue

            v = relative_pos.transform.translation
            dist = np.sqrt(v.x**2 + v.y**2 + v.z**2)
            if dist < self.neighbour_radius:
                if not self.collision_avoidance:
                    continue
                # Flip the horizontal component and leave z alone, so the drone
                # backs straight out of whichever direction it was heading.
                # Reversing x and y doesn't change the length of vec, so r from
                # above is still correct and the bump_threshold check below still
                # behaves normally.
                #
                # This breaks rather than continuing on purpose. If two
                # neighbours were both inside the radius, a second flip would
                # undo the first and send the drone straight back in.
                vec = np.array([-vec[X], -vec[Y], vec[Z]])
                self.get_logger().warn(
                    f'collision avoidance: {targ} at {dist:.3f} m (< '
                    f'{self.neighbour_radius:.2f} m); reversing heading xy.',
                    throttle_duration_sec=1.0)
                break

        if r > self.bump_threshold:
            vec = vec / r
            self.heading_msg = Vector3(x=float(vec[X]), y=float(vec[Y]), z=float(vec[Z]))
            # Under teleop the heading is still computed and still drawn, but it
            # never goes out. The controller takes whatever arrives on
            # desired_heading, scales it by max_speed and commands it, so
            # publishing here would mean competing with the keyboard.
            if not self.teleop_enabled:
                self.heading_pub.publish(self.heading_msg)

        # self.get_logger().info(f'heading: x={vec[X]:.3f} y={vec[Y]:.3f} z={vec[Z]:.3f} | r={r:.3f}')

        if self.ran_vis and self.drone_world_pos is not None:
            self._display_ran_rviz(self.nodes, self.z, vec)

        self.prev_time = now

    def _get_targets_from_tf(self):
        targets = []
        try:
            self_trans = self.tf_buffer.lookup_transform('mocap', self.drone_name, rclpy.time.Time())
        except tf2_ros.TransformException:
            return targets

        self_t = self_trans.transform.translation
        self_pos = np.array([self_t.x, self_t.y, self_t.z])
        self.drone_world_pos = self_pos

        for other in self.other_drones:
            try:
                other_trans = self.tf_buffer.lookup_transform('mocap', other, rclpy.time.Time())
            except tf2_ros.TransformException:
                continue
            other_t = other_trans.transform.translation
            other_pos = np.array([other_t.x, other_t.y, other_t.z])
            relative = other_pos - self_pos
            polar = self._cartesian_to_polar_3D(np.array([relative]))[0]
            quality = self.target_quality_overrides.get(other, self.target_quality)
            targets.append([polar[MAG], polar[PHI], (polar[THETA]), quality])
        return targets


    def _display_ran_rviz(self, nodes, activations, heading_vec):
        stamp = rclpy.time.Time().to_msg()  # zero stamp -> tf2 uses latest available transform
        cart_nodes = np.array(self._polar_to_cartesian_3D(nodes)) * self.vis_radius

        nodes_marker = Marker()
        nodes_marker.header.stamp = stamp
        nodes_marker.header.frame_id = 'mocap'
        nodes_marker.ns = 'ran_nodes'
        nodes_marker.id = 0
        nodes_marker.type = Marker.POINTS
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = self.vis_point_size
        nodes_marker.scale.y = self.vis_point_size
        nodes_marker.pose.position.x = float(self.drone_world_pos[X])
        nodes_marker.pose.position.y = float(self.drone_world_pos[Y])
        nodes_marker.pose.position.z = float(self.drone_world_pos[Z]) + self.vis_z_offset
        nodes_marker.pose.orientation.w = 1.0

        z_range = self.vis_z_max - self.vis_z_min

        for i in range(len(cart_nodes)):
            nodes_marker.points.append(Point(x=float(cart_nodes[i, X]), y=float(cart_nodes[i, Y]), z=float(cart_nodes[i, Z])))
            t = (activations[i] - self.vis_z_min) / z_range
            nodes_marker.colors.append(self._activation_to_color(t))

        heading_marker = Marker()
        heading_marker.header.stamp = stamp
        heading_marker.header.frame_id = 'mocap'
        heading_marker.ns = 'ran_heading'
        heading_marker.id = 0
        heading_marker.type = Marker.ARROW
        heading_marker.action = Marker.ADD
        heading_marker.pose.position.x = float(self.drone_world_pos[X])
        heading_marker.pose.position.y = float(self.drone_world_pos[Y])
        heading_marker.pose.position.z = float(self.drone_world_pos[Z]) + self.vis_z_offset
        heading_marker.pose.orientation.w = 1.0
        heading_marker.scale.x = self.vis_point_size         #shaft diameter
        heading_marker.scale.y = self.vis_point_size * 2.0   #head diameter
        heading_marker.scale.z = self.vis_point_size * 3.0   #head length
        heading_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        heading_marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(heading_vec[X]) * self.vis_radius * 1.5,
                  y=float(heading_vec[Y]) * self.vis_radius * 1.5,
                  z=float(heading_vec[Z]) * self.vis_radius * 1.5),
        ]

        self.vis_pub.publish(MarkerArray(markers=[nodes_marker, heading_marker]))

    def _activation_to_color(self, t):
        t = float(np.clip(t, 0.0, 1.0))
        stops = RAN_VIS_COLORMAP
        seg = t * (len(stops) - 1)
        i = min(int(seg), len(stops) - 2)
        f = seg - i
        r = stops[i][0] + f * (stops[i + 1][0] - stops[i][0])
        g = stops[i][1] + f * (stops[i + 1][1] - stops[i][1])
        b = stops[i][2] + f * (stops[i + 1][2] - stops[i][2])
        return ColorRGBA(r=r, g=g, b=b, a=1.0)

    def _generate_nodes(self, n_sub):
        # A plain icosphere, matching what generate_kernel_cache.py builds. This
        # is only a fallback for the moment before the kernel cache loads, since
        # loading it overwrites self.nodes with the exact points baked into the
        # cache. It's here so the object always has a usable self.nodes even if
        # the cache load fails partway through. In practice only its length ever
        # gets used, by num_nodes.
        nodes = pv.Icosphere(radius=1.0, nsub=n_sub)
        return nodes.points


    def _generate_sensory_input(self, sphere_points, targets, kappa):
        num_nodes = len(sphere_points)
        b = np.zeros(num_nodes)

        for j in range(len(targets)):
            for i in range(num_nodes):
                target_point = np.array((targets[j][MAG], targets[j][PHI], targets[j][THETA]))
                alpha = self._geodesic_distance(sphere_points[i], target_point)
                b[i] += np.exp(kappa * (np.cos(alpha) - 1.0)) * targets[j][QUALITY]

        b *= (1/np.sqrt(num_nodes))
        return b

    def _geodesic_distance(self, point1, point2):
        alpha = np.arccos(np.clip(np.cos(point1[THETA]) * np.cos(point2[THETA]) + np.sin(point1[THETA]) * np.sin(point2[THETA]) * np.cos(point1[PHI] - point2[PHI]), -1, 1))
        return alpha

    def _rand_link_func(self, y, sigma, inv_sqrt_n):
        out = np.empty_like(y)
        for i in range(y.size):
            out[i] = np.random.normal(0.0, sigma) * inv_sqrt_n
        return out

    def _find_vel_avg_3D(self, points, activations):
        points = np.array(self._polar_to_cartesian_3D(points))
        total_weight = np.sum(activations)
        if total_weight == 0:
            return np.zeros(3)
        x_bar = np.sum(points[:, X] * activations) / total_weight
        y_bar = np.sum(points[:, Y] * activations) / total_weight
        z_bar = np.sum(points[:, Z] * activations) / total_weight
        return np.array((x_bar, y_bar, z_bar))

    def _cartesian_to_polar_3D(self, points):
        cpy = points.copy()
        for i in range(len(cpy)):
            x, y, z = cpy[i]
            r = np.sqrt(x**2 + y**2 + z**2)
            theta = np.arctan2(y, x)
            if r != 0.0:
                phi = np.arccos(z / r)
            else:
                phi = 0.0
            cpy[i] = r, theta, phi
        return cpy

    def _polar_to_cartesian_3D(self, points):
        cpy = points.copy()
        for i in range(len(cpy)):
            r, theta, phi = cpy[i]
            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)
            z = r * np.cos(phi)
            cpy[i] = x, y, z
        return cpy



def main(args=None):
    rclpy.init(args=args)

    spherical_ran_server = SphericalRANServer()

    time.sleep(spherical_ran_server.startup_delay)
    rclpy.spin(spherical_ran_server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spherical_ran_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
