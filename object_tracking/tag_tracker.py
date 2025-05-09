import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult,FloatingPointRange


class ArucoStatsPublisher(Node):
    def __init__(self):
        super().__init__('aruco_stats_publisher')

        # Camera intrinsic parameters
        self.u0 = 500  # Principal point
        self.v0 = 275
        self.lx = 455  # Focal length / pixel size
        self.ly = 455

        # State variables
        self.set_desired_point = False
        self.desired_points = {}
        self.current_points = {}
        self.selected_points = []

        # ROS publishers and subscribers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/bluerov2/camera/image', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/tagged_frames', 10)
        self.stats_pub = self.create_publisher(Float32MultiArray, '/aruco_stats', 10)
        self.publisher_velocity = self.create_publisher(Twist, 'camera_velocity', 10)
        self.publisher_robot_velocity = self.create_publisher(Twist, '/bluerov2/visual_tracker', 10)
        self.publisher = self.create_publisher(Float64MultiArray, 'tracked_point', 10)

        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Parameters
        self.gain_matrix = np.eye(6)  # Identity matrix
        
        # Define DoF names (linear first, then angular)
        self.dof_names = ['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw']
        
        # Declare all parameters with descriptive names
        for dof in self.dof_names:
            float_range = FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.0001)
            self.declare_parameter(f'{dof}_gain', 0.0, ParameterDescriptor(
                description= 'slider control gain for ' + dof,
                read_only=False,
                floating_point_range= [float_range],
                additional_constraints= 'must be between -1.0 and 1.0'
            ))

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.update_matrix_from_params()


        # OpenCV window for visualization
        cv2.namedWindow("ArUco Tag Tracker")
        cv2.setMouseCallback("ArUco Tag Tracker", self.click_detect)

        # Transformation constants
        # self.camera_to_robot_rotation = np.array([
        #     [0, 0, 1],
        #     [-1, 0, 0],
        #     [0, -1, 0]
        # ])
        # self.camera_to_robot_translation = np.array([0.0, 0.03, 0.172])

    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name.endswith('_gain') and param.name.split('_')[0] in self.dof_names:
                dof = param.name.split('_')[0]
                idx = self.dof_names.index(dof)
                self.gain_matrix[idx, idx] = param.value
        
        self.get_logger().info(f"Updated gains: {self.get_current_gains()}")
        return SetParametersResult(successful=True)

    def update_matrix_from_params(self):
        """Update all matrix elements from current parameters"""
        for i, dof in enumerate(self.dof_names):
            self.gain_matrix[i,i] = self.get_parameter(f'{dof}_gain').value


    def get_current_gains(self):
        """Return gains as a readable dict"""
        return {dof: self.gain_matrix[i,i] for i, dof in enumerate(self.dof_names)}
    

    def click_detect(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.set_desired_point = True
            self.desired_points = {}
            dict_point_des = {}
            for cx, cy, area, marker_id, corners in self.selected_points:
                    dict_point_des[marker_id] = (cx, cy)

            # sort the dict by key
            dict_point_des = dict(sorted(dict_point_des.items()))
            self.desired_points = dict_point_des

            self.get_logger().info(f"Desired points set: {self.desired_points}")

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
    
    def interaction_matrix(self, pts: list) -> np.ndarray:
        if len(pts) % 2 != 0:
            self.get_logger().error("Invalid number of points for interaction matrix")
            return np.empty((0, 6))

        num_pts = len(pts) // 2
        L = []
        Z = 0.5  # Assume constant depth for simplicity

        for i in range(num_pts):
            x = pts[2*i]
            y = pts[2*i+1]
            # Convert to meters
            x, y = self.convert2meter((x, y), self.u0, self.v0, self.lx, self.ly)
            
            L.append([
        [self.lx * (-1/Z), 0, self.lx * (x/Z), self.lx * (x * y), -self.lx * (1 + x**2), self.lx * y],
        [0, self.ly * (-1/Z), self.ly * (y/Z), self.ly * (1 + y**2), -self.ly * (x * y), -self.ly * x]])
    
        # Stack matrices for all points
        return np.vstack(L)
    

    def compute_velocity(self, L: np.ndarray, errors: np.ndarray) -> np.ndarray:
        return - np.linalg.pinv(L) @ errors



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
        upper = np.hstack((R, skew @ R))
        lower = np.hstack((np.zeros((3,3)), R))
        T = np.vstack((upper, lower))  # 6x6 matrix

        v_robot = T @ v_camera
        return v_robot
    


    def image_callback(self, msg: Image):
        # Convert to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.set_desired_point:
            print("set desired point", self.desired_points)
            self.set_desired_point = False

        # Detect markers
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        
        # draw desired points
        for marker_id, (cx, cy) in self.desired_points.items():
            # if marker_id in self.current_points:
            color = (255, 0, 255)
            cv2.circle(frame, (int(cx), int(cy)), 5, color, -1)

        if ids is not None:
            stats = []
            for corners, marker_id in zip(corners_list, ids.flatten()):
                pts = corners.reshape((4, 2))
                cx, cy = pts.mean(axis=0)
                area = cv2.contourArea(pts.astype(np.float32))
                stats.append((cx, cy, area, int(marker_id), corners))

            # Sort by area descending and pick the three largest
            picked = sorted(stats, key=lambda x: x[2], reverse=True)[:3]

            # Draw and annotate markers
            for cx, cy, area, marker_id, corners in stats:
                color = (0, 255, 0) if any(marker_id == p[3] for p in picked) else (100, 100, 100)
                pts = corners.reshape((4, 2)).astype(int)
                cv2.polylines(frame, [pts], True, color, 2)
                cv2.putText(frame, f"ID:{marker_id}", (int(cx), int(cy)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                # Draw center point
                cv2.circle(frame, (int(cx), int(cy)), 5, color, -1)

            # Filter for specific IDs and build output message
            desired_ids = {1,2,3,6,7,8}
            selected = [s for s in stats if s[3] in desired_ids]
            self.selected_points = selected
            msg_out = Float32MultiArray()
            msg_out.data = []


            for cx, cy, area, marker_id, corners in selected:
                msg_out.data.extend([marker_id, cx, cy])
                self.current_points[marker_id] = (cx, cy)

            # sort the dict by key
            self.current_points = dict(sorted(self.current_points.items()))

            # self.get_logger().info(f"Desired points: {self.desired_points}")
            self.stats_pub.publish(msg_out)
            # self.get_logger().info(f'Published stats: {msg_out.data}')

            # Compute and publish velocities if desired points are set
            msg_out_tracked = Float64MultiArray()
            if self.desired_points:
                minimum_detections = min(len(self.desired_points), len(self.current_points))
                if minimum_detections > 0:
                    # self.get_logger().info(f"current points: {self.current_points}")
                    # self.get_logger().info(f"current points arr: {self.current_points_arr}")
                    # self.get_logger().info(f"desired points: {self.desired_points}")
                    # Compute errors
                    errors = []
                    self.current_points_arr = []
                    msg_out_tracked.data = []
                    for marker_id in self.current_points:
                        if marker_id in self.desired_points:
                            cx, cy = self.current_points[marker_id]
                            des_cx, des_cy = self.desired_points[marker_id]
                            errors.extend([(cx - des_cx), (cy - des_cy)])
                            msg_out_tracked.data.extend([cx - des_cx, cy - des_cy])
                            self.current_points_arr.extend([cx, cy])
                    
                    # Publish tracked point errors
                    self.publisher.publish(msg_out_tracked)
                    # Compute interaction matrix
                    
                    L = self.interaction_matrix(self.current_points_arr)
                    # self.get_logger().info(f"Interaction matrix L: {L}")
                    # Compute errors
                    errors = np.array(errors)
                    # self.get_logger().info(f"Errors: {errors}")

                    # Compute velocity
                    v_camera = self.compute_velocity(L, errors)
                    v_robot = self.full_velocity_transform(v_camera)
                    #multiply by gain matrix
                    v_robot = self.gain_matrix @ v_robot
                    # self.get_logger().info(f"v_robot: {v_robot}")

                    # Publish camera velocity
                    cam_velocity = Twist()
                    cam_velocity.linear.x = v_camera[0]
                    cam_velocity.linear.y = v_camera[1]
                    cam_velocity.linear.z = v_camera[2]
                    cam_velocity.angular.x = v_camera[3]
                    cam_velocity.angular.y = v_camera[4]
                    cam_velocity.angular.z = v_camera[5]
                    self.publisher_velocity.publish(cam_velocity)


                    # Publish robot velocity
                    msg_out_robot = Twist()
                    # limit the velocity to -1, 1
                    v_robot = np.clip(v_robot, -0.5, 0.5)
                    self.get_logger().info(f"v_robot: {v_robot}")
                    msg_out_robot.linear.x = v_robot[0] #* self.thruster_gain
                    msg_out_robot.linear.y = v_robot[1] #* self.thruster_gain
                    msg_out_robot.linear.z = v_robot[2] #* self.thruster_gain
                    msg_out_robot.angular.x = v_robot[3]# * self.thruster_gain
                    msg_out_robot.angular.y = v_robot[4]# * self.thruster_gain
                    msg_out_robot.angular.z = v_robot[5]# * self.thruster_gain
                    
                    self.publisher_robot_velocity.publish(msg_out_robot)


        # Show and republish annotated image
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
