import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64 
import numpy as np
import csv
import os
from datetime import datetime

class EKFBeaconNode(Node):
    def __init__(self):
        super().__init__('ekf_beacon_node')


        self.declare_parameter('beacon_id', 0)
        self.beacon_id = self.get_parameter('beacon_id').value

        # Parámetros del filtro
        self.declare_parameter('process_noise', 0.001)
        self.declare_parameter('measurement_noise', 0.1)

        self.process_noise = self.get_parameter('process_noise').value
        self.measurement_noise = self.get_parameter('measurement_noise').value

        # Suscriptores
        self.create_subscription(Float64, f'/beacon_{self.beacon_id}/distance_to_target', self.distance_callback, 10)
        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, f'/beacon_{self.beacon_id}/pf_beacon_init', self.pf_callback, 10)

        # Publicador del estado estimado
        self.est_pub = self.create_publisher(PoseWithCovarianceStamped, f'/beacon_{self.beacon_id}/ekf_beacon_estimate', 10)

        # Creación de directorio y archivo CSV
        log_dir = os.path.expanduser('~/pf_logs') 
        os.makedirs(log_dir, exist_ok=True)  # Crear la carpeta si no existe
        self.csv_path = os.path.join(log_dir, f'pf_means_ekf_{self.beacon_id}.csv')


        # Crear y abrir el archivo CSV
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x_mean', 'y_mean', 'z_mean'])

        # Estado EKF (x, y, z) y su covarianza
        self.x = None
        self.Sigma = None

        # Ruido de proceso y de medida
        self.Q = np.eye(3) * self.process_noise    # Baja incertidumbre porque la baliza es estática
        self.R = self.measurement_noise ** 2           # Varianza del ruido de medición de distancia

        # Posición actual del dron
        self.drone_pos = None
        self.start_time2 = self.get_clock().now()

    def pf_callback(self, msg):
        data = msg.pose.pose.position
        self.x = np.array([data.x, data.y, data.z])        
        cov = np.array(msg.pose.covariance).reshape((6, 6))
        self.Sigma = cov[:3, :3]
        # self.get_logger().info(f"Posicion: {self.x}    Covariance: {self.Sigma}")
        self.get_logger().info(f"Arrancando EKF para baliza {self.beacon_id}")

    def odom_callback(self, msg):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        # self.get_logger().info(f"Posicion dron: {self.drone_pos}")

    def distance_callback(self, msg):
        
        if msg.data < 0.0:
            z = None
        else:
            z = msg.data  # distancia medida
            
        if self.x is None or self.Sigma is None or self.drone_pos is None or z is None:
            # self.get_logger().info("Esperando datos de la baliza y el dron...")
            return  # aún no tenemos todo

        h = np.linalg.norm(self.x - self.drone_pos)  # distancia estimada

        # Jacobiano H (derivada de la distancia respecto a x)
        delta = self.x - self.drone_pos
        dist = np.linalg.norm(delta)
        if dist < 1e-6:
            dist = 1e-6  # evitar división por cero
        H = delta.reshape(1, 3) / dist  # 1x3

        # Predicción (es estática)
        x_pred = self.x
        Sigma_pred = self.Sigma + self.Q

        # Innovación
        y = z - h
        S = H @ Sigma_pred @ H.T + self.R
        K = Sigma_pred @ H.T @ np.linalg.inv(S)  # 3x1

        # Corrección
        self.x = x_pred + (K.flatten() * y)
        self.Sigma = (np.eye(3) - K @ H) @ Sigma_pred

        # self.get_logger().info(f"Estado estimado: {self.x}    Covariance: {self.Sigma}")

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # o el frame adecuado
        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]
        msg.pose.pose.position.z = self.x[2]

        # Convertir self.Sigma (3x3) a la parte de posición de la matriz 6x6
        cov_full = np.zeros((6, 6))
        cov_full[:3, :3] = self.Sigma
        msg.pose.covariance = cov_full.flatten().tolist()

        self.est_pub.publish(msg)

        # Guardar en el CSV
        elapsed_time2 = (self.get_clock().now() - self.start_time2).nanoseconds * 1e-9
        self.csv_writer.writerow([elapsed_time2, self.x[0], self.x[1], self.x[2]])
        self.csv_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = EKFBeaconNode()
    rclpy.spin(node)
    node.csv_file.close()
    rclpy.shutdown()
