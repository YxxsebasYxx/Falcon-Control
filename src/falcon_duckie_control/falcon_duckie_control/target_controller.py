import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TargetController(Node):

    def __init__(self):
        super().__init__('target_controller')

        self.subscription = self.create_subscription(
            Odometry,
            '/target/odometry',
            self.target_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/duckie/cmd_vel',
            10
        )

        # Ganancias del controlador
        self.k_v = 0.8
        self.k_omega = 12.0

        # Distancia deseada al objetivo (metros)
        self.target_distance = 0.5

        # Umbrales mínimos de operación para tus motores
        self.min_speed = 0.80
        self.min_omega = 12.00

        # Máximos de seguridad
        self.max_speed = 1.00
        self.max_omega = 16.00

        self.last_detection_time = self.get_clock().now()
        self.timeout = 0.5  # Frenar si pasan más de 0.5s sin detección

        self.timer = self.create_timer(0.05, self.control_loop)
        self.x = None
        self.y = None

    def target_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.last_detection_time = self.get_clock().now()

    def control_loop(self):
        cmd = Twist()

        # Si no hay detección previa o transcurrió el timeout, detener
        if self.x is None or self.y is None:
            self.publisher.publish(cmd)
            return

        elapsed = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        if elapsed > self.timeout:
            self.publisher.publish(cmd)
            return

        distance = math.sqrt(self.x ** 2 + self.y ** 2)
        distance_error = distance - self.target_distance

        # Cálculo de velocidades proporcionales
        v = self.k_v * distance_error
        angle = math.atan2(self.y, self.x)
        omega = self.k_omega * angle

        # APLICACIÓN DE UMBRALES MÍNIMOS PARA MOTORES

        # Control Angular: Garantizar mínimo 12 rad/s si hay desvío angular
        if abs(angle) > 0.08:  # Desvío superior a ~4.5 grados
            omega_val = max(self.min_omega, abs(omega))
            omega_val = min(self.max_omega, omega_val)
            omega = math.copysign(omega_val, omega)
        else:
            omega = 0.0

        # Control Lineal: Garantizar mínimo 0.8 m/s si falta distancia
        if distance_error > 0.10:  # Más de 10 cm alejado del objetivo
            v = max(self.min_speed, min(self.max_speed, v))
        else:
            v = 0.0

        cmd.linear.x = v
        cmd.angular.z = omega
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.publisher.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()