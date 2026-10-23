import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from inference import InferencePipeline


class DroneDetectionControl(Node):

    def __init__(self):
        super().__init__('drone_detection_control')

        # Publica los comandos de movimiento del Duckie
        self.publisher = self.create_publisher(
            Twist,
            '/duckie/cmd_vel',
            10
        )

        # Parámetros de control
        self.confidence_threshold = 0.80
        self.forward_speed = 0.10  # m/s: velocidad inicial baja

        # Seguridad: detenerse si dejan de llegar detecciones
        self.last_valid_detection = 0.0
        self.detection_timeout = 0.5

        # Comprobar periódicamente si debe detenerse
        self.timer = self.create_timer(
            0.1,
            self.check_detection_timeout
        )

        self.get_logger().info(
            'Control de Duckie por detección de drones iniciado.'
        )

    def publish_velocity(self, linear_x, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        self.publisher.publish(msg)

    def process_predictions(self, predictions, video_frame):
        """
        Callback de Roboflow InferencePipeline.

        Se espera que predictions sea un diccionario que contenga
        una lista bajo la clave 'predictions'.
        """

        try:
            detections = predictions.get('predictions', [])

            # Buscar la detección de dron con mayor confianza
            best_confidence = 0.0

            for detection in detections:
                confidence = float(
                    detection.get('confidence', 0.0)
                )

                class_name = str(
                    detection.get('class', '')
                ).lower()

                # Ajusta este nombre a la clase real de tu modelo.
                if class_name == 'drone':
                    best_confidence = max(
                        best_confidence,
                        confidence
                    )

            # ¿Se detectó un dron con confianza suficiente?
            if best_confidence >= self.confidence_threshold:

                self.last_valid_detection = time.monotonic()

                self.publish_velocity(
                    self.forward_speed,
                    0.0
                )

                self.get_logger().info(
                    f'Dron detectado: {best_confidence:.1%}. '
                    f'Duckie avanzando.'
                )

            else:
                self.publish_velocity(0.0, 0.0)

                self.get_logger().info(
                    'No hay detección válida. Duckie detenido.'
                )

        except Exception as error:
            self.publish_velocity(0.0, 0.0)

            self.get_logger().error(
                f'Error procesando detecciones: {error}'
            )

    def check_detection_timeout(self):
        """
        Si no hay detecciones válidas recientes,
        ordena detener el Duckie.
        """

        if self.last_valid_detection == 0.0:
            self.publish_velocity(0.0, 0.0)
            return

        elapsed = time.monotonic() - self.last_valid_detection

        if elapsed > self.detection_timeout:
            self.publish_velocity(0.0, 0.0)


def main(args=None):

    rclpy.init(args=args)

    node = DroneDetectionControl()

    # Usa una variable de entorno para no escribir la clave
    # directamente en el código.
    import os

    api_key = os.environ.get('ROBOFLOW_API_KEY')

    if not api_key:
        node.get_logger().error(
            'Falta configurar la variable ROBOFLOW_API_KEY.'
        )

        node.destroy_node()
        rclpy.shutdown()
        return

    pipeline = None

    try:
        pipeline = InferencePipeline.init(
            model_id='drones_new/3',
            api_key=api_key,
            video_reference=0,
            on_prediction=node.process_predictions
        )

        # Inicia la inferencia y mantiene ROS 2 funcionando
        pipeline.start()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        node.get_logger().error(
            f'Error iniciando la cámara o el modelo: {error}'
        )

    finally:
        # Ordenar detenerse antes de terminar
        node.publish_velocity(0.0, 0.0)

        if pipeline is not None:
            try:
                pipeline.terminate()
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()