import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty
import time


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.publisher = self.create_publisher(
            Twist,
            '/duckie/cmd_vel',
            10
        )

        # Velocidades iniciales altas para vencer la fricción estática
        self.linear_speed = 0.80
        self.angular_speed = 12.50

        # Guardar configuración original y activar modo RAW permanente
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.last_key_time = time.time()
        self.key_timeout = 0.45  # 450 ms cubre el retardo de repetición del teclado
        self.current_twist = Twist()

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.print_instructions()

    def print_instructions(self):
        msg = (
            "\r\n========================================\r\n"
            "       DUCKIE KEYBOARD TELEOP\r\n"
            "========================================\r\n"
            "        W\r\n"
            "     A  S  D\r\n\r\n"
            "W : Avanzar        S : Retroceder\r\n"
            "A : Girar Izq      D : Girar Der\r\n"
            "SPACE : Detener    (Auto-stop al soltar)\r\n\r\n"
            "Q : + Velocidad    Z : - Velocidad\r\n"
            "CTRL+C : Salir\r\n"
            "========================================\r\n"
        )
        sys.stdout.write(msg)
        sys.stdout.flush()

    def get_key(self):
        if select.select([sys.stdin], [], [], 0.001)[0]:
            return sys.stdin.read(1)
        return None

    def timer_callback(self):
        key = self.get_key()
        now = time.time()

        if key is not None:
            self.last_key_time = now

            if key == 'w':
                self.current_twist.linear.x = self.linear_speed
                self.current_twist.angular.z = 0.0
            elif key == 's':
                self.current_twist.linear.x = -self.linear_speed
                self.current_twist.angular.z = 0.0
            elif key == 'a':
                self.current_twist.linear.x = 0.0
                self.current_twist.angular.z = self.angular_speed
            elif key == 'd':
                self.current_twist.linear.x = 0.0
                self.current_twist.angular.z = -self.angular_speed
            elif key == ' ':
                self.current_twist.linear.x = 0.0
                self.current_twist.angular.z = 0.0
            elif key == 'q':
                self.linear_speed += 0.05
                self.angular_speed += 0.10
                sys.stdout.write(
                    f"\r\nVelocidad: {self.linear_speed:.2f} m/s | "
                    f"Angular: {self.angular_speed:.2f} rad/s\r\n"
                )
                sys.stdout.flush()
            elif key == 'z':
                self.linear_speed = max(0.10, self.linear_speed - 0.05)
                self.angular_speed = max(0.20, self.angular_speed - 0.10)
                sys.stdout.write(
                    f"\r\nVelocidad: {self.linear_speed:.2f} m/s | "
                    f"Angular: {self.angular_speed:.2f} rad/s\r\n"
                )
                sys.stdout.flush()
            elif key == '\x03':  # CTRL+C
                raise KeyboardInterrupt
        else:
            # Si transcurren más de 450 ms sin recibir teclas, detiene el movimiento
            if now - self.last_key_time > self.key_timeout:
                self.current_twist.linear.x = 0.0
                self.current_twist.angular.z = 0.0

        self.publisher.publish(self.current_twist)

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Enviar comando cero antes de destruir el contexto
        try:
            stop = Twist()
            node.publisher.publish(stop)
        except Exception:
            pass

        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()