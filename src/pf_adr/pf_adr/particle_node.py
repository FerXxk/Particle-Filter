import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import numpy as np
import csv
import os
from datetime import datetime

class BeaconParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        # Parámetro beacon_id
        self.declare_parameter('beacon_id', 0)
        self.beacon_id = self.get_parameter('beacon_id').value

        # Parámetros configurables
        self.declare_parameter('num_particles', 5000)
        self.declare_parameter('sigma', 0.02)
        self.declare_parameter('noise_std', 0.02)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('init_x_range', [-1.0, 1.0])
        self.declare_parameter('init_y_range', [-1.0, 1.0])
        self.declare_parameter('init_z_range', [0.0, 1.0])
        self.declare_parameter('noise_pos_drone',[0.003, 0.003, 0.002])
        self.declare_parameter('noise_dist',0.003)
        

        
        self.num_particles = self.get_parameter('num_particles').value
        self.sigma = self.get_parameter('sigma').value
        self.noise_std = self.get_parameter('noise_std').value
        self.radius = self.get_parameter('radius').value
        self.init_x_range = self.get_parameter('init_x_range').value
        self.init_y_range = self.get_parameter('init_y_range').value
        self.init_z_range = self.get_parameter('init_z_range').value
        self.noise_pos_drone = self.get_parameter('noise_pos_drone').value
        self.noise_dist = self.get_parameter('noise_dist').value

        self.name_fichero = ('archivo_csv_' + str(self.beacon_id))
        # Creación de directorio y archivo CSV
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser('~/pf_logs')  
        os.makedirs(log_dir, exist_ok=True)  # Crear la carpeta si no existe
        self.csv_path = os.path.join(log_dir, f'pf_means_{self.beacon_id}.csv')

        # Crear y abrir el archivo CSV
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x_mean', 'y_mean', 'z_mean'])

        self.particles = self.initialize_particles2()
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.current_drone_position = None
        self.current_distance = None

        # Topic distance_to_target específico de la baliza
        distance_topic = f'/beacon_{self.beacon_id}/distance_to_target'

        # Subscripciones
        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Float64, distance_topic, self.distance_callback, 10)

        # Publicadores con topics específicos
        self.pose_pub = self.create_publisher(PoseStamped, f'/pf/beacon_{self.beacon_id}/estimate', 10)
        self.particles_pub = self.create_publisher(PoseArray, f'/pf/beacon_{self.beacon_id}/particles', 10)

        self.timer = self.create_timer(0.05, self.update_filter)
        self.get_logger().info(f'Filtro de partículas para baliza {self.beacon_id} inicializado.')

        self.start_time = self.get_clock().now()

    def initialize_particles2(self):
        phi = np.random.uniform(0, 2 * np.pi, self.num_particles)
        costheta = np.random.uniform(-1, 1, self.num_particles)
        u = np.random.uniform(0, 1, self.num_particles)
        theta = np.arccos(costheta)
        r = self.radius * u**(1/3)
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * costheta
        return np.stack([x, y, z], axis=-1)

    def odom_callback(self, msg):
        noise_drone = np.random.normal(0, self.noise_pos_drone, 3)
        self.current_drone_position = np.array([
            msg.pose.pose.position.x + noise_drone[0],
            msg.pose.pose.position.y + noise_drone[1],
            msg.pose.pose.position.z + noise_drone[2]
        ])

    def distance_callback(self, msg):
        if msg.data < 0.0:
            self.current_distance = None
    
        else:
            noise_distance = np.random.normal(0, self.noise_dist)
            self.current_distance = msg.data + noise_distance

    def update_filter(self):
        if self.current_drone_position is None or self.current_distance is None:
            return

        diffs = self.particles - self.current_drone_position
        dists = np.linalg.norm(diffs, axis=1)

        self.weights = np.exp(-0.5 * ((dists - self.current_distance) ** 2) / self.sigma**2)
        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

        if self.effective_sample_size() < self.num_particles / 2:
            self.resample_particles()

        self.publish_estimate()
        self.publish_particles()

    def effective_sample_size(self):
        return 1.0 / np.sum(np.square(self.weights))

    def resample_particles(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0
        indices = np.searchsorted(cumulative_sum, np.random.rand(self.num_particles))
        self.particles = self.particles[indices]
        noise = np.random.normal(0, self.noise_std, self.particles.shape)
        self.particles += noise
        self.weights.fill(1.0 / self.num_particles)

    def publish_estimate(self):
        mean = np.average(self.particles, axis=0, weights=self.weights)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = mean[0]
        msg.pose.position.y = mean[1]
        msg.pose.position.z = mean[2]
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

        # Guardar en el CSV
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.csv_writer.writerow([elapsed_time, mean[0], mean[1], mean[2]])
        self.csv_file.flush()

    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = p[2]
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.particles_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BeaconParticleFilter()
    rclpy.spin(node)
    node.destroy_node()
    node.csv_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()