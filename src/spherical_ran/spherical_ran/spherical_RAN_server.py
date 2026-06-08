import os
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3
import pyvista as pv
from rclpy.duration import Duration as RCLDuration

#vicon imports

RAN_UPDATE_RATE = 50    #hz
X = MAG = 0
Y = PHI = 1     #phi = xy angle
Z = THETA = 2   #theta = z axis angle
QUALITY = 3

class SphericalRANServer(Node):

    def __init__(self):
        super().__init__('spherical_RAN_server')
        self.declare_parameter('drone_name', 'cf01') # declare
        self.declare_parameter('beta', 1.0)
        self.declare_parameter('v', 0.5)
        self.declare_parameter('sigma', 1.5)
        self.declare_parameter('kappa', 20.0)
        self.declare_parameter('u', 15.0)
        self.declare_parameter('J', 5.0)
        self.declare_parameter('n_sub', 3)


        self.drone_name = self.get_parameter('drone_name').value    #drone name parameter
        self.beta = self.get_parameter('beta').value
        self.v = self.get_parameter('v').value
        self.sigma = self.get_parameter('sigma').value
        self.kappa = self.get_parameter('kappa').value
        self.u = self.get_parameter('u').value
        self.J = self.get_parameter('J').value
        self.n_sub = self.get_parameter('n_sub').value

        self.nodes = self._generate_nodes(self.n_sub)
        self.num_nodes = len(self.nodes)
        self.dt = 0.0
        self.z = np.zeros(self.num_nodes)
        
        cache_path = 'src/spherical_ran/spherical_ran/kernel_cache.npz'
        try:
            data = np.load(cache_path)
            if (data['n_sub'] != self.n_sub or data['J'] != self.J or data['v'] != self.v):
                raise ValueError('cached kernel parameters do not match current parameters')

            self.nodes = data['nodes']
            self.alphas = data['alphas']
            self.M = data['M']
            self.get_logger().info(f'Loaded cached kernel from {cache_path}')

        except (FileNotFoundError, ValueError, OSError) as e:
            self.get_logger().fatal(
                f'kernel cache unusable ({e}). Run generate_kernel_cache.py and restart.')
            raise RuntimeError(f'kernel cache unusable: {e}') from e

        ##Replace with vicon later        
        target1 = [1.0, 0, np.pi/2, 20.0]   #distance, phi, theta, quality
        target2 = [1.0, (-np.pi), np.pi/2, 20.0]   #distance, phi, theta, quality
        self.targets = [target1, target2]

        self.heading_pub = self.create_publisher(Vector3, f'{self.drone_name}/desired_heading', 10)
        self.timer = self.create_timer((1.0/RAN_UPDATE_RATE), self.update)
        self.heading_msg = Vector3()
        self.heading_msg.x = 0.0
        self.heading_msg.y = 0.0
        self.heading_msg.z = 0.0
        self.prev_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        self.dt = (now - self.prev_time).nanoseconds * 1e-9

        b = self._generate_sensory_input(self.nodes, self.targets, self.kappa)
        noise = self._rand_link_func(self.z, self.sigma * np.sqrt(self.dt), 1.0/np.sqrt(self.num_nodes))
        self.z = self.z + self.dt * (-(self.z) + np.tanh(self.u * (self.M @ self.z) + b - self.beta) - np.tanh(-self.beta) + noise)

        vec = self._find_vel_avg_3D(self.nodes, self.z)
        norm = np.linalg.norm(vec)
        if norm > 0:
            vec = vec / norm
        self.heading_msg = Vector3(x=float(vec[X]), y=float(vec[Y]), z=float(vec[Z]))
        self.heading_pub.publish(self.heading_msg)
        self.get_logger().info(f'publishing {self.heading_msg}')

        self.prev_time = now
        
    def _generate_nodes(self, n_sub):
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
        actv = activations.copy()
        points = np.array(self._polar_to_cartesian_3D(points))
        total_weight = np.sum(activations)
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

    rclpy.spin(spherical_ran_server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spherical_ran_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()