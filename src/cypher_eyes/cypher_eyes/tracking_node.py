import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time

class ServoSerialNode(Node):
    def __init__(self):
        super().__init__('servo_serial_node')

        # Subscribe to object detection topic
        self.subscription = self.create_subscription(
            Point,
            '/detected_object',
            self.object_callback,
            10
        )

        # Initialize serial communication with Arduino
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 250000, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info("Serial connection established with Arduino.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # Frame parameters (same as your detection model)
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2

        self.tolerance = 20  # pixel tolerance
        self.kp_x = 1.0
        self.kp_y = 1.0

    def object_callback(self, msg: Point):
        if self.ser is None:
            self.get_logger().warn("Serial port not initialized.")
            return

        # Calculate pixel error (Center - Object)
        error_x = (self.frame_center_x - msg.x) * self.kp_x
        error_y = (self.frame_center_y - msg.y) * self.kp_y

        # Ignore small jitter within tolerance
        if abs(error_x) < self.tolerance:
            error_x = 0
        if abs(error_y) < self.tolerance:
            error_y = 0

        # Send as text (not binary)
        send_str = f"{int(error_x)},{int(error_y)}\n"
        try:
            self.ser.write(send_str.encode())
            self.get_logger().info(f"Sent to Arduino: {send_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
