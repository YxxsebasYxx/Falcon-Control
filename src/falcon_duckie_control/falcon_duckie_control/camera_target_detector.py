import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class CameraTargetDetector(Node):

    def __init__(self):
        super().__init__('camera_target_detector')

        self.publisher = self.create_publisher(
            Odometry,
            '/target/odometry',
            10
        )

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara.")

        # Rango HSV para color Amarillo/Naranja (Ajustable)
        self.lower_color = np.array([10, 80, 80])
        self.upper_color = np.array([35, 255, 255])

        self.k_distance = 120.0

        # Filtro de suavizado (Exponential Moving Average)
        self.smooth_x = None
        self.smooth_y = None
        self.alpha = 0.35  # Factor de suavizado (0.1 = muy suave, 1.0 = sin filtro)

        self.timer = self.create_timer(0.05, self.process_frame)
        self.get_logger().info("Detector OpenCV iniciado con ventana de diagnóstico.")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.resize(frame, (640, 480))
        height, width, _ = frame.shape
        center_x_frame = width / 2.0

        # Conversión a HSV y filtrado
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

        # Operaciones morfológicas para eliminar ruido en la imagen
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        target_detected = False

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            # Filtrar por área mínima del contorno para ignorar objetos pequeños
            if area > 400:
                ((cx, cy), radius) = cv2.minEnclosingCircle(c)

                # Estimación de coordenadas x e y
                raw_x = self.k_distance / max(radius, 1.0)
                raw_y = -((cx - center_x_frame) / width) * raw_x

                # Aplicar filtro EMA para evitar saltos en la estimación
                if self.smooth_x is None:
                    self.smooth_x = raw_x
                    self.smooth_y = raw_y
                else:
                    self.smooth_x = (
                        self.alpha * raw_x + (1.0 - self.alpha) * self.smooth_x
                    )
                    self.smooth_y = (
                        self.alpha * raw_y + (1.0 - self.alpha) * self.smooth_y
                    )

                # Publicar mensaje en /target/odometry
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'camera'
                odom_msg.pose.pose.position.x = float(self.smooth_x)
                odom_msg.pose.pose.position.y = float(self.smooth_y)
                self.publisher.publish(odom_msg)

                target_detected = True

                # Dibujar realimentación en pantalla
                cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"X: {self.smooth_x:.2f}m | Y: {self.smooth_y:.2f}m",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        if not target_detected:
            self.smooth_x = None
            self.smooth_y = None
            cv2.putText(
                frame,
                "BUSCANDO OBJETIVO...",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

        # Mostrar video original y máscara en paralelo para calibración de color
        cv2.imshow("Camara Duckie", frame)
        cv2.imshow("Mascara de Color (Ajuste HSV)", mask)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()