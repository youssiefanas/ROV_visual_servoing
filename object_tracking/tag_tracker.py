import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


class ArucoStatsPublisher(Node):
    def __init__(self):
        super().__init__('aruco_stats_publisher')

        self.u0 = 320 # principle point
        self.v0 = 240 # principle point
        self.lx = 455 # focal length/ pixel size
        self.ly = 455 # focal length/ pixel size
        self.kud = 0.00683 
        self.kdu = -0.01424
        self.depth= 0
        self.set_desired_point = False



        
        # Image subscription & processed-image publisher
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/video_topic', self.image_callback, 10)
        self.image_pub = self.create_publisher(
            Image, '/tagged_frames', 10)
        
        # Publisher for marker stats: [cx1, cy1, area1, cx2, cy2, area2, cx3, cy3, area3]
        self.stats_pub = self.create_publisher(
            Float32MultiArray, '/aruco_stats', 10)
        
        self.publisher = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        # creat a publisher for the cam velocity
        self.publisher_velocity = self.create_publisher(Twist, 'camera_velocity', 10)

        # pub robot velocity
        self.publisher_robot_velocity = self.create_publisher(Twist, 'visual_tracker', 10)

        cv2.setMouseCallback("Buoy Tracking", self.click_detect)


        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()


        self.lambda_gain = 0.3
        self.thruster_gain = 1.0

        # create parameter callback
        self.config = {}
        self.declare_and_set_params()
        self.add_on_set_parameters_callback(self.set_parameters_callback)

    def update_parameters(self):
        self.lambda_gain = float(self.config.get('lambda_gain', 0.3))
        self.get_logger().info(f">>>>>>>>>>>>>>>>>>>>>>>>>>> lambda_gain updated to: {self.lambda_gain}")
        self.thruster_gain = float(self.config.get('thruster_gain', 1.0))
        self.get_logger().info(f"=========================== thruster_gain updated to: {self.thruster_gain}")
    
    def set_parameters_callback(self, params):
        for param in params:
            self.config[param.name] = param.value

        self.update_parameters()
        return SetParametersResult(successful=True)

    def declare_and_set_params(self):
        # declare
        self._declare_and_fill_map('lambda_gain', 0.3, 'lambda gain for camera velocity', self.config)
        self._declare_and_fill_map('thruster_gain', 1.0, 'thruster gain for the robot velocity', self.config)

        self.update_parameters()
        # pass

    def _declare_and_fill_map(self, key, val, description, map):
        param = self.declare_parameter(
            key, val, ParameterDescriptor(description=description))
        map[key] = param.value
        # pass

        
    def click_detect(self, event,flags):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.set_desired_point = True
            self.get_logger().info("get center point")

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
    
    def interaction_matrix(self, pts):
        num_pts = len(pts)/2
        L = []
        Z =1

        for i in range(num_pts):
            x = pts[2*i]
            y = pts[2*i+1]
            # Convert to meters
            x, y = self.convert2meter((x, y), self.u0, self.v0, self.lx, self.ly)
            
            L_in = np.array([
        [self.lx * (-1/Z), 0, self.lx * (x/Z), self.lx * (x * y), -self.lx * (1 + x**2), self.lx * y],
        [0, self.ly * (-1/Z), self.ly * (y/Z), self.ly * (1 + y**2), -self.ly * (x * y), -self.ly * x]])
        

        
            L.append(L_in)
    
        # Stack matrices for all points
        return np.vstack(L)
    

    def compute_velocity(self,L, errors, lambda_gain):
        # Compute the pseudo-inverse of L
        L_pseudo_inv = np.linalg.pinv(L)

        # Compute the velocity
        v = -lambda_gain * L_pseudo_inv @ errors

        return v



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
        R = np.array([
           [ 0, 0, 1],
           [-1,0,0],
            [0, -1,0]
        ])

        # 2nd choice
        # R = np.array([
        #    [ 0, 0, 1],
        #    [1,0,0],
        #     [0, 1,0]
        # ])

        # 3rd choice
        # R = np.array([
        #    [ 0, -1, 0],
        #    [0,0,-1],
        #     [1, 0,0]
        # ])

        v_c_lin = v_camera[:3]
        v_c_ang = v_camera[3:]

        # Build 6x6 transformation matrix
        upper = np.hstack((R, np.zeros((3,3))))
        lower = np.hstack((skew @ R, R))
        T = np.vstack((upper, lower))  # 6x6 matrix

        v_robot = T @ v_camera
        return v_robot
    



    def image_callback(self, msg: Image):
        # Convert to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        stats = []  # will hold (cx, cy, area) for each detected marker
       
        if ids is not None:
            stats = []
            # Build list of (cx, cy, area, id, corners)
            for corners, marker_id in zip(corners_list, ids.flatten()):
                pts = corners.reshape((4, 2))
                cx, cy = pts.mean(axis=0)
                area = cv2.contourArea(pts.astype(np.float32))
                stats.append((cx, cy, area, int(marker_id), corners))

            # Sort by area descending
            stats.sort(key=lambda x: x[2], reverse=True)

            # Pick the three largest
            picked = stats[:3]

            # Draw and annotate
            for cx, cy, area, marker_id, corners in stats:
                is_picked = any(marker_id == p[3] for p in picked)
                color = (0, 255, 0) if is_picked else (100, 100, 100)
                pts = corners.reshape((4, 2)).astype(int)
                cv2.polylines(frame, [pts], True, color, 2)
                cv2.putText(frame, f"ID:{marker_id}", (int(cx), int(cy)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Publish IDs of the three largest
            # Filter for specific IDs
            desired_ids = {1, 2, 9}
            selected = [s for s in stats if s[3] in desired_ids]

            # Build output message with cx, cy per ID
            msg_out = Float32MultiArray()
            msg_out.data = []
            # self.desired_points = 2 by 3 matrix 
            self.desired_points = []
            

            for cx, cy, area, marker_id, corners in selected:
                msg_out.data.extend([marker_id,cx, cy])  # ID followed by center x, y
                #current_points
                if self.set_desired_point:
                    self.desired_points.append([cx, cy])
            self.set_desired_point = False
            self.stats_pub.publish(msg_out)
            self.get_logger().info(f'Published stats: {msg_out.data}')

            # get the errors
            if len(self.desired_points) > 0:
                # convert to meters
                self.desired_points = np.array(self.desired_points).reshape(-1)
                # get the interaction matrix
                L = self.interaction_matrix(self.desired_points)
                # get the errors
                errors = self.desired_points- np.array([cx, cy]).reshape(-1)
                # compute the velocity
                v_camera = self.compute_velocity(L, errors, self.lambda_gain)
                # transform to robot frame
                v_robot = self.full_velocity_transform(v_camera)

                # publish the velocity
                msg_out = Float64MultiArray()
                msg_out.data = v_robot.tolist()
                self.publisher.publish(msg_out)

                # publish the camera velocity
                msg_out = Twist()
                msg_out.linear.x = v_camera[0]
                msg_out.linear.y = v_camera[1]
                msg_out.linear.z = v_camera[2]
                msg_out.angular.x = v_camera[3]
                msg_out.angular.y = v_camera[4]
                msg_out.angular.z = v_camera[5]
                
                self.publisher_velocity.publish(msg_out)



            

        # show & republish annotated image
        cv2.imshow("ArUco Tag Tracker", frame)
        cv2.waitKey(1)
        out_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.image_pub.publish(out_img)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoStatsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
