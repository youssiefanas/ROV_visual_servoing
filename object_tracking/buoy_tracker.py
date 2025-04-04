import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

class BuoyTracker(Node):
    def __init__(self):
        super().__init__('buoy_tracker')

        # Declare parameters with a default list of integers
        # self.declare_parameter('lower_hsv', [0, 144, 117]) #buoy red  # Default: [146, 128, 101] # pablo bottle
        # self.declare_parameter('upper_hsv', [33, 220, 255]) #buoy red  # Default: [179, 255, 255] # pablo bottle
        self.lower_hsv = np.array([0, 144, 117], dtype=np.uint8)
        self.upper_hsv = np.array([33, 220, 255], dtype=np.uint8)
        self.u0 = 320 # principle point
        self.v0 = 240 # principle point
        self.lx = 455 # focal length/ pixel size
        self.ly = 455 # focal length/ pixel size
        self.kud = 0.00683 
        self.kdu = -0.01424

        self.get_hsv = False
        self.set_desired_point = False
        self.set_desired_area = False
        self.mouseX = 0
        self.mouseY = 0
        self.rect_x1 = 0
        self.rect_y1 = 0
        self.rect_x2 = 0
        self.rect_y2 = 0
        
        self.z_ref = 0.1
        self.area_ref = 0.4
        self.z_des = self.z_ref

        # ROS2 subscribers & publishers
        self.subscription = self.create_subscription(
            Image,
            'video_topic',  # This is the topic your camera publisher uses
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Float64MultiArray, 'tracked_point', 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        # Minimum detection size
        self.min_width = 20
        self.min_height = 20

        # Set the initial HSV values from parameters
        # self.set_hsv_thresholds()

        # Parameter update callback to check and update the HSV thresholds at runtime
        # self.create_timer(1.0, self.update_hsv_values)

        self.get_logger().info("Buoy Tracker Node Initialized.")
        
        cv2.namedWindow("Buoy Tracking")
        cv2.setMouseCallback("Buoy Tracking", self.click_detect)

    # def set_hsv_thresholds(self):
    #     # Access HSV parameters using .get_parameter_value().get_parameter_value()
    #     lower_hsv_param = self.get_parameter('lower_hsv').get_parameter_value().integer_array_value
    #     upper_hsv_param = self.get_parameter('upper_hsv').get_parameter_value().integer_array_value

    #     # Convert these arrays into numpy arrays for further processing
    #     self.lower_hsv = np.array(lower_hsv_param, dtype=np.uint8)
    #     self.upper_hsv = np.array(upper_hsv_param, dtype=np.uint8)

    # def update_hsv_values(self):
    #     # Update HSV values from parameters at runtime
    #     self.set_hsv_thresholds()


    def click_detect(self, event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and flags != cv2.EVENT_FLAG_SHIFTKEY:
                self.get_hsv = True
                self.mouseX, self.mouseY = x, y
                print(self.upper_hsv)

            # if flags == cv2.EVENT_FLAG_SHIFTKEY:
            #     print("Shift key pressed")
            if event == cv2.EVENT_LBUTTONDOWN and flags == cv2.EVENT_FLAG_SHIFTKEY:
                self.set_desired_point = True
                self.mouseX, self.mouseY = x, y
                # self.z_des = self.depth
            if event == cv2.EVENT_LBUTTONDOWN and flags == cv2.EVENT_FLAG_CTRLKEY:
                self.set_desired_area = True
                self.rect_x1, self.rect_y1 = x, y
                if event == cv2.EVENT_LBUTTONUP:
                    self.rect_x2, self.rect_y2 = x, y
                    #self.set_desired_area = False
                    self.area_interested = self.frame[self.rect_y1:self.rect_y2, self.rect_x1:self.rect_x2]
                    print("Desired area: ")

    def get_hsv_bounds(self):
        tolerance = np.array([10, 40, 40])
        delta_h, delta_s, delta_v = tolerance

        hsv_color = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)[self.mouseY, self.mouseX]

        H, S, V = hsv_color  # Extract HSV values

        # Apply tolerance
        lower_bound = np.array([max(0, H - delta_h), max(0, S - delta_s), max(0, V - delta_v)], dtype=np.uint8)
        upper_bound = np.array([min(180, H + delta_h), min(255, S + delta_s), min(255, V + delta_v)], dtype=np.uint8)

        return lower_bound, upper_bound

    def desired_point(self):
        return self.mouseX, self.mouseY

    def convert2meter(self, pt, u0, v0, lx, ly):
        return (float(pt[0]) - u0) / lx, (float(pt[1]) - v0) / ly

    def convertOnePoint2meter(self, pt):
        return self.convert2meter(pt, self.u0, self.v0, self.lx, self.ly)

    def convertListPoint2meter(self, points):
        if np.shape(points)[0] > 1:
            n = int(np.shape(points)[0] / 2)
            point_reshaped = np.array(points).reshape(n, 2)
            point_meter = [self.convert2meter(pt, self.u0, self.v0, self.lx, self.ly) for pt in point_reshaped]
            return np.array(point_meter).reshape(-1)
    
    def interaction_matrix (self, point, depth):
        L = []

        Z = depth
        x,y  = self.convert2meter(point, self.u0, self.v0, self.lx, self.ly)
        
        L =  np.array([
        [self.lx * (-1/Z), 0, self.lx * (x/Z), self.lx * (x * y), -self.lx * (1 + x**2), self.lx * y],
        [0, self.ly * (-1/Z), self.ly * (y/Z), self.ly * (1 + y**2), -self.ly * (x * y), -self.ly * x]])
        

        return np.vstack(L)
    
    def compute_camera_velocity(self, L, error, depth_err, lambda_gain=0.5):
        L_pinv = np.linalg.pinv(L)  # Moore-Penrose pseudo-inverse
        # Compute velocity command
        velocity = -lambda_gain * L_pinv @ error
        v_z = -lambda_gain * depth_err
        return velocity, v_z
    
            


    def remove_reflections(self, frame, mask):
        """
        Detects the waterline and removes reflections above it.
        The waterline is modeled as a polynomial curve, taking into account camera motion.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        edges = cv2.Canny(gray, 50, 150)  # Detect edges

        # Visualize edges to see if waterline is detected
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

        # Detect horizontal lines using Hough Transform or other methods to detect edges
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)

        if lines is not None:
            # Find the points of the detected lines (water surface points)
            points = []
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if y1 == y2:  # Only consider horizontal lines (water surface)
                        points.append((x1, y1))

            if len(points) > 0:
                # Sort points by their y-coordinates (row number)
                points = sorted(points, key=lambda x: x[1])

                # Separate x and y coordinates
                x_points = np.array([p[0] for p in points])
                y_points = np.array([p[1] for p in points])

                # Visualize the detected points on the image (for debugging)
                for p in points:
                    cv2.circle(frame, p, 3, (0, 255, 0), -1)

                if len(x_points) > 5:  # Make sure there are enough points to fit a curve
                    # Fit a polynomial curve to the detected points (waterline)
                    poly_coeffs = np.polyfit(y_points, x_points, deg=2)  # 2nd-degree polynomial (quadratic)

                    # Generate the fitted waterline
                    y_fit = np.linspace(min(y_points), max(y_points), num=500)  # Generate y values for fitting
                    x_fit = np.polyval(poly_coeffs, y_fit)  # Get corresponding x values from the polynomial

                    # Visualize the fitted polynomial curve on the image
                    for i in range(len(y_fit)):
                        cv2.circle(frame, (int(x_fit[i]), int(y_fit[i])), 2, (255, 0, 0), -1)

                    # Convert the fitted points back to integer coordinates
                    x_fit_int = np.array(np.round(x_fit), dtype=int)
                    y_fit_int = np.array(np.round(y_fit), dtype=int)

                    # Create a mask to remove reflections above the fitted curve
                    for i in range(len(x_fit_int)):
                        if y_fit_int[i] < frame.shape[0]:  # Ensure within image bounds
                            mask[y_fit_int[i]:, x_fit_int[i]] = 0  # Mask out the area above the waterline
                else:
                    self.get_logger().warn("Not enough points detected for polynomial fitting.")
            else:
                self.get_logger().warn("No horizontal lines detected for waterline.")
        else:
            self.get_logger().warn("No lines detected by Hough Transform.")

        return mask

    def full_velocity_transform(self, v_camera):
        """
        Apply full 6x6 velocity transformation from camera to robot frame.

        :param v_camera: (6,) numpy array: [vx, vy, vz, wx, wy, wz]
        :param R: 3x3 rotation matrix from camera to robot
        :param t: 3x1 translation vector (camera origin w.r.t robot frame, expressed in camera frame)
        :return: v_robot: (6,) numpy array
        """
        x, y, z = np.array([0.1, 0.0, 0.05])

        skew = np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
        ])

        theta = np.radians(90)
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0, 0, 1]
        ]) 

        v_c_lin = v_camera[:3]
        v_c_ang = v_camera[3:]

        # Build 6x6 transformation matrix
        upper = np.hstack((R, np.zeros((3,3))))
        lower = np.hstack((skew @ R, R))
        T = np.vstack((upper, lower))  # 6x6 matrix

        v_robot = T @ v_camera
        return v_robot
    

    def image_callback(self, msg):
        try:
            
            """
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Mask using predefined HSV range
            mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
            """

            # Convert ROS2 image to OpenCV format
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.get_hsv:
                self.lower_hsv, self.upper_hsv = self.get_hsv_bounds()
                self.get_hsv = False

            if self.set_desired_point:
                print("Desired point: ", self.desired_point())
                self.set_desired_point = False

            # Convert to HSV
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

            # Mask using predefined HSV range
            mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

            # **Apply water reflection removal**
            # mask = self.remove_reflections(frame, mask)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            x,y,self.area = 0,0,0
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                x_meter, y_meter = self.convertOnePoint2meter((x, y))

                if radius > 10:
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(self.frame, center, radius, (0, 255, 0), 2)
                    cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
                    self.area = cv2.contourArea(largest_contour)
                

            # Publish center coordinates
            msg = Float64MultiArray()
            msg.data = [float(x), float(y), float(self.area)]
            self.publisher.publish(msg)

            self.error = []
            self.error.extend([x - self.mouseX, y - self.mouseY])
            self.depth = (self.area * self.z_ref)/self.area_ref
            depth_err = self.depth - self.z_des

            L  = self.interaction_matrix((x, y),self.depth)
            v_cam = self.compute_camera_velocity(L,self.error,depth_err)
            print(v_cam)
            # print (self.full_velocity_transform(v_cam))



            # Display windows
            cv2.imshow("Buoy Tracking", self.frame)  # This will show the frame with bounding box
            cv2.imshow("Mask", mask)  # Show the mask to debug how well the buoy is detected
            cv2.waitKey(1)  # Wait for a key event

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BuoyTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # Ensure windows are closed when ROS2 shuts down

if __name__ == '__main__':
    main()