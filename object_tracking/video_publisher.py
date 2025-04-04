import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        # Publisher for camera frames
        self.publisher = self.create_publisher(Image, 'video_topic', 10)
        
        # OpenCV video capture
        self.cap = cv2.VideoCapture(0)#"/home/hans/Downloads/red_buoy.mp4")#0) #select input
        self.bridge = CvBridge()

        # Timer to publish frames
        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
        
        # Convert frame to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Publish
        self.publisher.publish(img_msg)
        self.get_logger().info("Published video frame")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
