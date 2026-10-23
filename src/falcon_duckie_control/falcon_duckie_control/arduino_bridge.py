import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        # Declare ROS 2 parameters with defaults
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.10)
        self.declare_parameter('gain', 180.0)

        # Retrieve parameter values
        port_name = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.gain = self.get_parameter('gain').get_parameter_value().double_value

        # Open serial connection
        try:
            self.serial_port = serial.Serial(
                port_name,
                baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Arduino bridge connected on {port_name} at {baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {port_name}: {e}')
            raise e

        self.subscription = self.create_subscription(
            Twist,
            '/duckie/cmd_vel',
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        # Differential drive kinematic conversion
        v_left = v - omega * self.wheel_base / 2.0
        v_right = v + omega * self.wheel_base / 2.0

        # Convert velocity to PWM values
        pwm_left = int(v_left * self.gain)
        pwm_right = int(v_right * self.gain)

        # Clamp values to standard PWM range [-255, 255]
        pwm_left = max(-255, min(255, pwm_left))
        pwm_right = max(-255, min(255, pwm_right))

        command = f"{pwm_left},{pwm_right}\n"

        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.write(command.encode())

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b'0,0\n')
                self.serial_port.close()
            except Exception as e:
                self.get_logger().warn(f'Error closing serial port: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()