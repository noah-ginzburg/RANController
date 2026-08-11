import os
import time

import rclpy
from rclpy.node import Node
import numpy as np
import pyvista as pv
from scipy.spatial import SphericalVoronoi
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import ColorRGBA

import tf2_ros
from rclpy.duration import Duration as RCLDuration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray


RAN_UPDATE_RATE = 50    #hz
RAN_VIS_RADIUS = 0.1    #m, radius of the displayed icosphere
RAN_VIS_POINT_SIZE = 0.007   #m, size of each node marker
RAN_VIS_Z_OFFSET = 0.015    #m, shift of the icosphere along the drone's z axis (tune to recenter on the body)
RAN_VIS_Z_MIN = -1.0    #activation value mapped to the cold end of the colormap
RAN_VIS_Z_MAX = 2.0     #activation value mapped to the hot end of the colormap
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


class SphericalRANServerLloyd(Node):
    """Clone of SphericalRANServer (spherical_RAN_server.py) that uses an
    icosphere relaxed via spherical Lloyd's algorithm (Centroidal Voronoi
    Tessellation) instead of a bare icosphere. Fixes the icosphere's 12 fixed
    valence-5 vertices, which are ~16% denser than the rest of the mesh --
    see the Lloyd-relaxation cell in mean_field_model_3d.ipynb (same
    algorithm, kept in sync manually) and generate_kernel_cache_lloyd.py.
    Only _generate_nodes and the kernel cache path differ from the original
    -- see that file for the plain icosphere version, which this was cloned
    from and is left untouched.
    """

    def __init__(self):
        super().__init__('spherical_RAN_server_lloyd')
        self.declare_parameter('drone_name', 'cf01')
        self.declare_parameter('ran_vis', True)

        self.declare_parameter('beta', 1.5)
        self.declare_parameter('v', 0.3)
        self.declare_parameter('sigma', 0.85)
        self.declare_parameter('kappa', 7.0)
        self.declare_parameter('u', 35.0)
        self.declare_parameter('rate', 1.0)
        # How concentrated the activation has to be, on a 0-1 scale, before we
        # trust its direction enough to publish a heading -- see class
        # docstring / the resultant-length comment in update() for what this
        # number actually measures.
        self.declare_parameter('bump_threshold', 0.3)
        self.declare_parameter('n_sub', 3)
        self.declare_parameter('all_drones', [''])
        self.declare_parameter('target_quality', 20.0)
        self.declare_parameter('target_names', [''])
        self.declare_parameter('target_qualities', [0.0])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.drone_name = self.get_parameter('drone_name').value
        self.ran_vis = self.get_parameter('ran_vis').value
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

        all_drones = self.get_parameter('all_drones').value
        self.other_drones = [d for d in all_drones if d != self.drone_name]

        self.nodes = self._generate_nodes(self.n_sub)
        self.num_nodes = len(self.nodes)
        self.dt = 0.0
        self.z = np.zeros(self.num_nodes)

        cache_path = 'src/spherical_ran/spherical_ran/kernel_cache_lloyd.npz'
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
                f'kernel cache unusable ({e}). Run generate_kernel_cache_lloyd.py and restart.')
            raise RuntimeError(f'kernel cache unusable: {e}') from e

        self.targets = []
        # None until tf answers. Starting this at the origin drew the whole
        # visualization on the floor at (0,0,0) for as long as the drone's
        # transform took to show up -- every tf lookup below fails quietly.
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
        self.timer = self.create_timer((1.0/RAN_UPDATE_RATE), self.update)
        self.heading_msg = Vector3()
        self.heading_msg.x = 0.0
        self.heading_msg.y = 0.0
        self.heading_msg.z = 0.0
        self.prev_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        # self.dt = (now - self.prev_time).nanoseconds * 1e-9
        self.dt = 0.4

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
        # `vec` is a weighted average of unit-length node positions, so its
        # own length is the "mean resultant length" R from directional
        # statistics: R=1 means every active node points the same way (a
        # sharp, formed bump); R=0 means the activations cancel out (still
        # flat/blended, no committed direction yet). This replaces an
        # arbitrary scalar cutoff on raw activation with the actual
        # concentration of the population vector -- see the class docstring.
        r = np.linalg.norm(vec)

        for targ in self.other_drones:
            try:
                relative_pos = self.tf_buffer.lookup_transform(f'{self.drone_name}', targ, rclpy.time.Time())
            except tf2_ros.TransformException:
                continue

            v = relative_pos.transform.translation
            dist = np.sqrt(v.x**2 + v.y**2 + v.z**2)
            if dist < 0.3:
                # vec = np.array([-vec[X], -vec[Y], vec[Z]])
                test=1

        if r > self.bump_threshold:
            vec = vec / r
            self.heading_msg = Vector3(x=float(vec[X]), y=float(vec[Y]), z=float(vec[Z]))
            self.heading_pub.publish(self.heading_msg)

        # self.get_logger().info(f'heading: x={vec[X]:.3f} y={vec[Y]:.3f} z={vec[Z]:.3f} | r={r:.3f}')


        if self.drone_world_pos is not None:
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
        cart_nodes = np.array(self._polar_to_cartesian_3D(nodes)) * RAN_VIS_RADIUS

        nodes_marker = Marker()
        nodes_marker.header.stamp = stamp
        nodes_marker.header.frame_id = 'mocap'
        nodes_marker.ns = 'ran_nodes'
        nodes_marker.id = 0
        nodes_marker.type = Marker.POINTS
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = RAN_VIS_POINT_SIZE
        nodes_marker.scale.y = RAN_VIS_POINT_SIZE
        nodes_marker.pose.position.x = float(self.drone_world_pos[X])
        nodes_marker.pose.position.y = float(self.drone_world_pos[Y])
        nodes_marker.pose.position.z = float(self.drone_world_pos[Z]) + RAN_VIS_Z_OFFSET
        nodes_marker.pose.orientation.w = 1.0

        z_range = RAN_VIS_Z_MAX - RAN_VIS_Z_MIN

        for i in range(len(cart_nodes)):
            nodes_marker.points.append(Point(x=float(cart_nodes[i, X]), y=float(cart_nodes[i, Y]), z=float(cart_nodes[i, Z])))
            t = (activations[i] - RAN_VIS_Z_MIN) / z_range
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
        heading_marker.pose.position.z = float(self.drone_world_pos[Z]) + RAN_VIS_Z_OFFSET
        heading_marker.pose.orientation.w = 1.0
        heading_marker.scale.x = RAN_VIS_POINT_SIZE         #shaft diameter
        heading_marker.scale.y = RAN_VIS_POINT_SIZE * 2.0   #head diameter
        heading_marker.scale.z = RAN_VIS_POINT_SIZE * 3.0   #head length
        heading_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        heading_marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(heading_vec[X]) * RAN_VIS_RADIUS * 1.5,
                  y=float(heading_vec[Y]) * RAN_VIS_RADIUS * 1.5,
                  z=float(heading_vec[Z]) * RAN_VIS_RADIUS * 1.5),
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
        # Icosphere, then relaxed via spherical Lloyd's algorithm (Centroidal
        # Voronoi Tessellation) -- see generate_kernel_cache_lloyd.py (same
        # algorithm, kept in sync manually) and the Lloyd-relaxation cell in
        # mean_field_model_3d.ipynb. This is only a fallback used before the
        # kernel cache loads below (which overwrites self.nodes with the
        # exact relaxed points baked into the cache) -- kept so the object
        # always has a valid self.nodes even if the cache load fails partway.
        icosphere = pv.Icosphere(radius=1.0, nsub=n_sub)
        return self._lloyd_relax_sphere(icosphere.points.copy())

    def _spherical_triangle_area(self, a, b, c):
        # Solid angle subtended by triangle a,b,c as seen from the sphere's
        # center == the triangle's area on the unit sphere (Van Oosterom &
        # Strackee formula) -- much simpler than L'Huilier's theorem.
        numer = np.abs(np.dot(a, np.cross(b, c)))
        denom = 1.0 + np.dot(a, b) + np.dot(b, c) + np.dot(c, a)
        return 2.0 * np.arctan2(numer, denom)

    def _lloyd_relax_sphere(self, points, n_iter=30):
        points = points / np.linalg.norm(points, axis=1, keepdims=True)
        for _ in range(n_iter):
            sv = SphericalVoronoi(points, radius=1.0, center=np.zeros(3))
            sv.sort_vertices_of_regions()
            new_points = np.empty_like(points)
            for i, region in enumerate(sv.regions):
                verts = sv.vertices[region]
                gen = points[i]
                centroid = np.zeros(3)
                for j in range(len(verts)):
                    a, b = verts[j], verts[(j + 1) % len(verts)]
                    area = self._spherical_triangle_area(gen, a, b)
                    tri_mid = gen + a + b
                    centroid += area * (tri_mid / np.linalg.norm(tri_mid))
                new_points[i] = centroid / np.linalg.norm(centroid)
            points = new_points
        return points

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

    spherical_ran_server = SphericalRANServerLloyd()

    time.sleep(10.0)
    rclpy.spin(spherical_ran_server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spherical_ran_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
