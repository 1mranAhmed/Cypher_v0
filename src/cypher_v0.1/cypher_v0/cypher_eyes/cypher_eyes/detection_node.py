import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
from geometry_msgs.msg import Point

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.publisher_ = self.create_publisher(Point, '/detected_object', 10)

        # Load YOLO model (path inside your package)
        self.model = YOLO("/home/imran/ros2_ws/src/my_robot_controller/my_robot_controller/best.pt")

        # Open webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            raise RuntimeError("Camera not available")

        # Timer for callback (~20 Hz)
        self.timer = self.create_timer(0.05, self.detect_callback)

        # Calibration constant for distance estimation
        self.CALIBRATION_K = 10.0

        # Frame size (for center calculation)
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_center = (self.frame_width // 2, self.frame_height // 2)

        self.get_logger().info("Object detection node initialized.")

    def estimate_distance(self, bbox_height):
        """Estimate distance based on bounding box height."""
        if bbox_height > 0:
            return self.CALIBRATION_K / bbox_height
        else:
            return None

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not received.")
            return

        results = self.model(frame, verbose=False)

        for r in results:
            boxes = r.boxes
            if len(boxes) == 0:
                continue

            # Pick the most confident detection only
            best_box = max(boxes, key=lambda b: float(b.conf[0]))
            x1, y1, x2, y2 = map(int, best_box.xyxy[0])
            conf = float(best_box.conf[0])

            bbox_height = y2 - y1
            distance_m = self.estimate_distance(bbox_height)
            if not distance_m:
                continue

            # Compute object center
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # Publish as a geometry_msgs/Point
            msg = Point()
            msg.x = float(cx)
            msg.y = float(cy)
            msg.z = float(distance_m)
            self.publisher_.publish(msg)

            # Visualization (optional)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"{distance_m:.2f}m ({conf:.2f})",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)
            cv2.imshow("YOLO Detection", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        """Ensure clean resource release."""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
