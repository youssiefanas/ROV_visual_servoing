import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class BuoyTracker(Node):
    def __init__(self):
        super().__init__('buoy_tracker')

        # Declare parameters with a default list of integers
        # self.declare_parameter('lower_hsv', [0, 144, 117]) #buoy red  # Default: [146, 128, 101] # pablo bottle
        # self.declare_parameter('upper_hsv', [33, 220, 255]) #buoy red  # Default: [179, 255, 255] # pablo bottle
        self.lower_hsv = np.array([0, 144, 117], dtype=np.uint8)
        self.upper_hsv = np.array([33, 220, 255], dtype=np.uint8)
        self.u0 = 500 # principle point
        self.v0 = 275 # principle point
        self.lx = 455 # focal length/ pixel size
        self.ly = 455 # focal length/ pixel size
        self.kud = 0.00683 
        self.kdu = -0.01424
        self.depth= 0
        self.get_hsv = False
        self.set_desired_point = False
        self.set_desired_area = False
        self.mouseX = 0
        self.mouseY = 0
        self.rect_x1 = 0
        self.rect_y1 = 0
        self.rect_x2 = 0
        self.rect_y2 = 0
        
        # self.z_ref = 1
        self.area_ref = 6000
        self.buoy_size = 3.15 # Size of the buoy in meters
        self.z_des = 0.5  # Desired depth in meters

        # ROS2 subscribers & publishers
        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera/image',  # This is the topic your camera publisher uses
            # 'video_topic',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        # creat a publisher for the cam velocity
        self.publisher_velocity = self.create_publisher(Twist, 'camera_velocity', 10)

        # pub robot velocity
        self.publisher_robot_velocity = self.create_publisher(Twist, '/bluerov2/visual_tracker', 10)

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

        # TODO: reconfigurable param for lambda gain
        # declare and set params
        # add in set parameters callback

        self.lambda_gain = 0.5
        self.thruster_gain = 0.5

        # create parameter callback
        self.config = {}
        self.declare_and_set_params()
        self.add_on_set_parameters_callback(self.set_parameters_callback)

    def update_parameters(self):
        self.lambda_gain = float(self.config.get('lambda_gain', 0.5))
        self.get_logger().info(f">>>>>>>>>>>>>>>>>>>>>>>>>>> lambda_gain updated to: {self.lambda_gain}")
        self.thruster_gain = float(self.config.get('thruster_gain', 0.5))
        self.get_logger().info(f"=========================== thruster_gain updated to: {self.thruster_gain}")
    
    def set_parameters_callback(self, params):
        for param in params:
            self.config[param.name] = param.value

        self.update_parameters()
        return SetParametersResult(successful=True)

    def declare_and_set_params(self):
        # declare
        self._declare_and_fill_map('lambda_gain', 0.5, 'lambda gain for camera velocity', self.config)
        self._declare_and_fill_map('thruster_gain', 0.5, 'thruster gain for the robot velocity', self.config)

        self.update_parameters()
        # pass

    def _declare_and_fill_map(self, key, val, description, map):
        param = self.declare_parameter(
            key, val, ParameterDescriptor(description=description))
        map[key] = param.value
        # pass

        
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
                print("update hsv",self.upper_hsv)

            # if flags == cv2.EVENT_FLAG_SHIFTKEY:
            #     print("Shift key pressed")
            if event == cv2.EVENT_LBUTTONDOWN and flags == cv2.EVENT_FLAG_SHIFTKEY:
                self.set_desired_point = True
                self.mouseX, self.mouseY = x, y
                self.z_des = self.depth
                # print("Desired point: ", self.mouseX, self.mouseY)
                # print("Depth: ", self.depth)
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

        Z = 1 # max(depth, 0.001)
        x,y  = self.convert2meter(point, self.u0, self.v0, self.lx, self.ly)
        
        L =  np.array([
        [self.lx * (-1/Z), 0, self.lx * (x/Z), self.lx * (x * y), -self.lx * (1 + x**2), self.lx * y],
        [0, self.ly * (-1/Z), self.ly * (y/Z), self.ly * (1 + y**2), -self.ly * (x * y), -self.ly * x]])
        

        return np.vstack(L)
    
    def compute_camera_velocity(self, L, error, depth_err, lambda_gain=1):
        L_pinv = np.linalg.pinv(L)  # Moore-Penrose pseudo-inverse
        # Compute velocity command
        velocity = - L_pinv @ error
        v_z = lambda_gain * depth_err
        velocity[2] = v_z  # Set the vertical velocity to the computed depth error
        # linear velocity gain 
        velocity[0:2] = 0.5 * velocity[0:2]
        # angular velocity gain
        velocity[3:6] = lambda_gain * velocity[3:6]
        return velocity
    
            


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

        # check rotation matrix and translation vector

        # x, y, z = np.array([0.172, 0, 0.03])
        x, y, z = np.array([0.0, 0.03, 0.172])

        skew = np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
        ])

        # 1st choice
        # R = np.array([
        #    [ 0, 0, 1],
        #    [-1,0,0],
        #     [0, -1,0]
        # ])

        # 2nd choice
        R = np.array([
           [ 0, 0, 1],
           [1,0,0],
            [0, 1,0]
        ])

        # 3rd choice
        # R = np.array([
        #    [ 0, -1, 0],
        #    [0,0,-1],
        #     [1, 0,0]
        # ])

        # Build 6x6 transformation matrix
        # upper = np.hstack((R, np.zeros((3,3))))
        # lower = np.hstack((skew @ R, R))

        upper = np.hstack((R, skew@R))
        lower = np.hstack((np.zeros((3,3)), R))
        T = np.vstack((upper, lower))  # 6x6 matrix

        v_robot = T @ v_camera
        return v_robot
    

    def image_callback(self, msg):
        self.get_logger().info("Received image")
        try:
            
            """
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Mask using predefined HSV range
            mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
            """

            # Convert ROS2 image to OpenCV format
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #self.get_logger().info(f'depth = {self.depth}')
            


            if self.get_hsv:
                self.lower_hsv, self.upper_hsv = self.get_hsv_bounds()
                print("Lower HSV: ", self.lower_hsv)
                print("Upper HSV: ", self.upper_hsv)
                self.get_hsv = False

            if self.set_desired_point:
                print("Desired point: ", self.desired_point())
                print("Depth: ", self.depth)
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
                    cv2.circle(self.frame, (self.mouseX,self.mouseY), 5, (255, 0, 255), 2)
                    cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
                    self.area = cv2.contourArea(largest_contour)
                    # print("Area: ", self.area)
                

            if self.area < 1e-9:
                self.area = 1e-9

            self.error = []
            self.error.extend([x - self.mouseX, y - self.mouseY])
            #self.error= self.convertListPoint2meter(self.error)
            self.get_logger().error(f"error: {self.error}")
            
            #self.get_logger().info(f'{self.area}')
            self.depth = np.sqrt(3000/ self.area)
            
            # print("buoy size: ", self.buoy_size)
            #print("Depth: ", self.depth)
            depth_err = self.depth - self.z_des
            #depth_err = self.convert2meter((depth_err,0), self.u0, self.v0, self.lx, self.ly)[0]

            # Publish center coordinates
            msg = Float64MultiArray()
            msg.data = [float(x - self.mouseX), float(y-self.mouseY), float(depth_err), float(x), float(y)]
            # current position of the buoy
            # msg.dat


            self.publisher.publish(msg) # current pose oof the buoy



            L  = self.interaction_matrix((x, y),self.depth)
            self.get_logger().error(f"interaction matrix: {L}")
            v_cam = self.compute_camera_velocity(L,self.error,depth_err,lambda_gain=self.lambda_gain)
            self.get_logger().error(f"cam vel: {v_cam}")


            # print("Velocity in camera frame: ", v_cam)
            # publish camera velocity
            velocity_msg = Twist()
            velocity_msg.linear.x = v_cam[0]
            velocity_msg.linear.y = v_cam[1]
            velocity_msg.linear.z = v_cam[2]    
            velocity_msg.angular.x = v_cam[3]
            velocity_msg.angular.y = v_cam[4]
            velocity_msg.angular.z = v_cam[5]   
            self.publisher_velocity.publish(velocity_msg)
            
            # print("desired point: ", self.desired_point())
            # print("error: ", self.error)
            robot_vel = self.full_velocity_transform(v_cam)

            # TODO: scale robot_vel with thruster_gain

            # publish robot velocity
            robot_velocity_msg = Twist()
            robot_velocity_msg.linear.x = self.thruster_gain * robot_vel[0]
            robot_velocity_msg.linear.y = 0.5 * robot_vel[1]
            robot_velocity_msg.linear.z = self.thruster_gain * robot_vel[2]
            robot_velocity_msg.angular.x = self.thruster_gain * robot_vel[3]
            robot_velocity_msg.angular.y = self.thruster_gain * robot_vel[4]
            robot_velocity_msg.angular.z = self.thruster_gain * robot_vel[5]
            
            self.publisher_robot_velocity.publish(robot_velocity_msg)





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