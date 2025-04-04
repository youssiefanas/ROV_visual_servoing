import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np

class AprilTagTracker(Node):
    def __init__(self):
        super().__init__('apriltag_tracker')
        
        # Subscribe to incoming video frames
        self.subscription = self.create_subscription(
            Image,
            '/video_topic',  # Change this if necessary
            self.image_callback,
            10
        )
        
        # Publisher for processed frames with detected tags
        self.publisher = self.create_publisher(Image, '/tagged_frames', 10)
        
        self.bridge = CvBridge()
        
        # Initialize AprilTag detector with multiple families
        options = apriltag.DetectorOptions(families="tag36h11,tag25h9,tag16h5,tagStandard41h12")
        self.detector = apriltag.Detector(options)

        # Flag for publishing detection message
        self.tag_detected = False

    def image_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        
        # Convert ROS image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale for better detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.convertScaleAbs(gray, alpha=2, beta=50)

        # Detect AprilTags
        detections = self.detector.detect(gray)
        
        # If a tag is detected, update flag and publish a message
        if detections:
            if not self.tag_detected:
                self.get_logger().info("At least one tag detected!")
                self.tag_detected = True
                # Here you can publish a custom message or trigger a specific event
                # For now, we'll just log it for simplicity.

        # Draw the detected tags on top of the original frame
        for detection in detections:
            # Draw bounding box around detected tag
            for i in range(4):
                pt1 = tuple(detection.corners[i].astype(int))
                pt2 = tuple(detection.corners[(i + 1) % 4].astype(int))
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            
            # Draw tag ID at the center
            cX, cY = map(int, detection.center)
            cv2.putText(frame, str(detection.tag_id), (cX - 10, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the processed frame with tracked edges
        cv2.imshow("AprilTag Tracker", frame)
        cv2.waitKey(1)  # This is required for OpenCV to update the window

        # Convert the processed frame back to ROS message and publish
        tagged_frame_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(tagged_frame_msg)
        self.get_logger().info('Published processed frame')

    def destroy_node(self):
        cv2.destroyAllWindows()  # Ensure OpenCV windows are closed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()