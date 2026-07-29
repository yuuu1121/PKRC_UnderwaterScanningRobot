import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class UARTReceiverNode(Node):
    def __init__(self, port):
        super().__init__('uart_receiver_node')
        self.port = port
        self.baudrate = 9600
        self.timeout = 0
        self.publisher = self.create_publisher(String, 'uart_data', 10)
        self.serial_port = None

        try:
            # Initialize the serial port
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.get_logger().info(f"Opened UART port: {self.port}")

            # Reset RX buffer
            self.serial_port.reset_input_buffer()
            self.get_logger().info("RX buffer initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to open port {self.port}: {e}")

        # Timer to continuously read data
        self.timer = self.create_timer(0.1, self.read_uart_data)

    def read_uart_data(self):
        if self.serial_port:
            try:
                data = self.serial_port.read(2)  # Read 2 bytes at a time
                if data:
                    try:
                        received_char = data.decode(errors='ignore').strip()  # Decode and strip whitespace
                        msg = String()
                        msg.data = received_char
                        self.publisher.publish(msg)
                        self.get_logger().info(f"Published: {received_char}")
                    except UnicodeDecodeError:
                        self.get_logger().warn(f"Received invalid data: {data}")
            except Exception as e:
                self.get_logger().error(f"Error reading data: {e}")

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Closed UART port.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    uart_port = '/dev/ttyUSB0'  # Replace with your UART port name
    uart_receiver_node = UARTReceiverNode(uart_port)

    try:
        rclpy.spin(uart_receiver_node)
    except KeyboardInterrupt:
        uart_receiver_node.get_logger().info("Node interrupted by user.")
    finally:
        uart_receiver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
