import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, Int32MultiArray
import csv
import os

class BeaconActivityMonitor(Node):
    def __init__(self):
        super().__init__('beacon_activity_monitor')
        self.start_time = self.get_clock().now()
        
        # Parámetros configurables
        self.declare_parameter('num_beacons', 5)
        self.num_beacons = self.get_parameter('num_beacons').value

        self.declare_parameter('timeout_sec', 1.0)
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.declare_parameter('total_particles', 5000)
        self.total_particles = self.get_parameter('total_particles').value

        self.last_seen = [0.0] * self.num_beacons
        self.subscribers = []

        # Creación de directorio y archivo CSV
        log_dir = os.path.expanduser('~/pf_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, 'pf_particles.csv')

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        header = ['timestamp'] + [f'beacon_{i}' for i in range(self.num_beacons)]
        self.csv_writer.writerow(header)

        for i in range(self.num_beacons):
            topic_name = f'/beacon_{i}/distance_to_target'
            sub = self.create_subscription(
                Float64,
                topic_name,
                lambda msg, i=i: self.distance_callback(msg, i),
                10
            )
            self.subscribers.append(sub) 

        self.publisher_count = self.create_publisher(Int32, '/active_beacons_count', 10)
        self.publisher_ids = self.create_publisher(Int32MultiArray, '/active_beacons_ids', 10)
        self.publisher_distribution = self.create_publisher(Int32MultiArray, '/beacon_particle_distribution', 10)

        self.timer = self.create_timer(0.5, self.check_active_beacons)

        self.get_logger().info(f'Monitor iniciado para {self.num_beacons} balizas.')

    def distance_callback(self, msg, beacon_id):
        if 0.0 < msg.data < 10.0 and msg.data != float('inf'):
            self.last_seen[beacon_id] = self.get_clock().now().nanoseconds * 1e-9 

    def check_active_beacons(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        active_ids = [
            i for i, t in enumerate(self.last_seen)
            if (now - t) < self.timeout_sec
        ]
        active_count = len(active_ids)

        # Publicar cantidad
        msg_count = Int32()
        msg_count.data = active_count
        self.publisher_count.publish(msg_count)

        # Publicar lista de IDs activas
        msg_ids = Int32MultiArray()
        msg_ids.data = active_ids
        self.publisher_ids.publish(msg_ids)

        # Calcular distribución de partículas
        distribution = [0] * self.num_beacons
        per_beacon = 0
        if active_count > 0:
            per_beacon = self.total_particles // active_count
            for i in active_ids:
                distribution[i] = per_beacon

        msg_dist = Int32MultiArray()
        msg_dist.data = distribution
        self.publisher_distribution.publish(msg_dist)

        # Guardar en CSV
        elapsed_time = now - (self.start_time.nanoseconds * 1e-9)
        self.csv_writer.writerow([elapsed_time] + distribution)
        self.csv_file.flush()

        self.get_logger().info(f'Balizas activas: {active_ids} → {per_beacon} partículas cada una.')

def main(args=None):
    rclpy.init(args=args)
    node = BeaconActivityMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        node.csv_file.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
