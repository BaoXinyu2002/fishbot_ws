# from laser_tracker import *
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
# from geometry_msgs.msg import Twist, Pose, Point
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum

import sys
import cv2
import numpy as np
import pyrealsense2 as rs
import time

# import socket

# class MySocketClass:
#     def __init__(self, recv_port, send_port, my_ip, other_ip):
#         # host = socket.gethostname()
#         # # 创建接收用的socket
#         # self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         # self.recv_socket.bind(("35.3.33.37", recv_port))
#         # self.recv_socket.listen(1)

#         # print("wait for sockets")
#         # time.sleep(5)

#         # 创建发送用的socket
#         self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.send_socket.connect(("35.3.202.217", send_port))

#     def send_data(self, data):
#         # 发送数据
#         self.send_socket.sendall(data.encode())

#     def receive_data(self):
#         # 接收数据
#         connection, address = self.recv_socket.accept()
#         data = connection.recv(1024)
#         connection.close()
#         return data.decode()

class LaserTracker():
    def __init__(self, hls_thres:dict, color):
        self.hue_min = hls_thres["hue_min"]
        self.hue_max = hls_thres["hue_max"]
        self.sat_min = hls_thres["sat_min"]
        self.sat_max = hls_thres["sat_max"]
        self.l_min = hls_thres["l_min"]
        self.l_max = hls_thres["l_max"]
        self.color = color

        self.previous_position = None
        self.centers = [] # center[0]->width  center[1]->height
        self.prev_len = 0
        self.curr_len = 0
        # self.center_cnt = 0
        self.last_time = 0

    def clear_trail(self):
        self.centers = []
        self.previous_position = None

    def find_blob(self, frame, mask, mask_with_color):
        """Use blob detector to find laser point"""
        # Create the detector with the parameters
        detector = cv2.SimpleBlobDetector.create()
        params = detector.getParams()

        # Filter by color (white)
        params.filterByColor = False
        # params.blobColor = 255

        # Set Circularity filtering parameters 
        params.filterByCircularity = False
        params.minCircularity = 0.5
        
        # Set Convexity filtering parameters 
        params.filterByConvexity = False
        params.minConvexity = 0.8

        # Set inertia filtering parameters 
        params.filterByInertia = False
        params.minInertiaRatio = 0.8

        # Filter by Area
        params.filterByArea = True
        params.minArea = 5

        detector.setParams(params)
        
        # Detect blobs 
        keypoints = detector.detect(mask)

        color_contours, _ = cv2.findContours(mask_with_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(frame, color_contours, -1, (255,0,0), 1)
        # cv2.imshow("Green Mask", mask_with_color)

        for keypoint in keypoints:
            for contour in color_contours:
                if cv2.pointPolygonTest(contour, keypoint.pt, False) > 0:
                    center = (int(keypoint.pt[0]), int(keypoint.pt[1]))
                    # cv2.circle(frame, center, 10, (255, 0, 255), 2)
                    # if self.previous_position:
                        # cv2.line(self.trail, self.previous_position, center,
                        #                 (255, 255, 255), 2)
                        # cv2.add(self.trail, frame, frame)
                    self.previous_position = center
                    self.centers.append([center[0], center[1]])
                    print(self.color, ": ", self.centers)
                    return
        
        # max_y = 0
        # center_x = 0
        # center_y = 0
        # for keypoint in keypoints:
        #     x, y = keypoint.pt  
        #     if y > center_y:
        #         center_y = y
        #         center_x = x

        # if center_y > self.cam_height/3:
        #     center = int(center_x), int(center_y)
        #     print(center)
        #     if self.previous_position:
        #         cv2.line(self.trail, self.previous_position, center,
        #                         (255, 255, 255), 2)

        #     cv2.add(self.trail, frame, frame)
        #     self.previous_position = center
            # self.centers.append([center[0], center[1]])

    def detect(self, frame):
        hls_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        
        # "red":{"hue_min":0, "hue_max":20, "l_min":60, 
        #            "l_max":180, "sat_min":127, "sat_max":255}, 
        #     "green":{"hue_min":40, "hue_max":80, "l_min":60, 
        #            "l_max":180, "sat_min":127, "sat_max":255}} 
        hls_thres = cv2.inRange(hls_img, (0, 200, 0), (180, 255, 255))
        if self.color == "green":
            hls_thres_with_color = cv2.inRange(hls_img, (40, 60, 127), (80, 180, 255))
        # hls_thres_with_color = cv2.morphologyEx(hls_thres_with_color, cv2.MORPH_DILATE, np.ones((7,7),np.uint8))
        elif self.color == "red":
            hls_thres_with_color = cv2.inRange(hls_img, (0, 150, 127), (15, 180, 255))
            tmp = cv2.inRange(hls_img, (165, 150, 127), (180, 180, 255))
            hls_thres_with_color = cv2.bitwise_or(hls_thres_with_color, tmp)
        hls_thres_with_color = cv2.morphologyEx(hls_thres_with_color, cv2.MORPH_DILATE, np.ones((7,7),np.uint8))
        
        self.find_blob(frame, hls_thres, hls_thres_with_color)
    
        return hls_thres
    
    def check_activate(self):
        """Check whether there is a clockwise circle"""
        # if (len(self.centers) < 10):
        #     return False
        # centers = np.array(self.centers)
        # left = np.mean(centers[:int(len(centers)/5), 0])
        # right = np.mean(centers[int(0.8*len(centers)):, 0])
        # if right > 480 and left < 160:
        #     return True
        # return False

        if self.curr_len != len(self.centers):
            self.last_time = time.time()

        self.curr_len = len(self.centers)

        if time.time() >= self.last_time+5.0 and self.curr_len==self.prev_len:
            self.centers.clear()
            self.prev_len = 0
            self.curr_len = 0

        self.prev_len = len(self.centers)

        if (len(self.centers) < 6):
            return False
        
        left = self.centers[0][0]
        right = self.centers[-1][0]
        if right > 500 and left < 140:
            return True
        return False

    def get_target_pos(self, depth_frame, depth_scale, depth_intrin):
        """Identify the target position when selected"""
        if len(self.centers) == 0:
            return None
        depth = depth_frame[self.centers[0][1], self.centers[0][0]].astype(float)
        dist = depth * depth_scale
        print(dist)

        camera_cord = rs.rs2_deproject_pixel_to_point(depth_intrin, [self.centers[0][0], self.centers[0][1]], dist)
        target_x = camera_cord[2]
        target_y = -camera_cord[0] # define left as positive

        return [target_x, target_y]

class States(Enum):
    """enums for states"""
    IDLE = 1
    SELECTED = 2
    NAVIGATING = 3
    ARRIVE = 4   
    FAILED = 5

class Tasks(Enum):
    """enums for tasks"""
    SPIN = 1
    DRIVE_SQUARE = 2

class Runner(Node):
    def __init__(self, recv_port, send_port, my_ip, other_ip):
        super().__init__("runner") #, namespace="ff1")
        self.cam_width = 640
        self.cam_height = 480
        self.hls_thres = {
            "red":{"hue_min":0, "hue_max":20, "l_min":60, 
                   "l_max":180, "sat_min":127, "sat_max":255}, 
            "green":{"hue_min":40, "hue_max":80, "l_min":60, 
                   "l_max":180, "sat_min":127, "sat_max":255}} 
        self.red_tracker = LaserTracker(self.hls_thres["red"], "red")
        self.green_tracker = LaserTracker(self.hls_thres["green"], "green")

        self.pipeline = None
        self.config = None
        self.align = None
        self.depth_scale = None
        # self.depth_intrin = None
        self.setup_camera()

        self.state = States.IDLE #States.IDLE
        self.task = None
        self.name = 'ff1'
        self.offset = {
            'ff0': [0.0, 0.0],
            'ff1': [0.0, 0.0]
        }
        self.cur_pose = [0.0, 0.0, 0.0]
        self.cur_x = 0.0 
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.rob_frame_target_position = None #[0.0, 0.0]
        self.laser_detected = False
        self.cur_color = None

        self.navigator = BasicNavigator()
        self.init_navigator()
        
        # self.tf_node = Node("map_base_link_frame_listener")
        self.target_frame = self.declare_parameter(
            "target_frame", "base_link").get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.from_frame_rel = self.target_frame
        self.to_frame_rel = "map"
        
        # self.start()
        # self.tf_timer = self.create_timer(0.5, self.on_tf_timer)
        self.timer = self.create_timer(0.1, self.on_timer)
        
        self.target_pos_publisher = self.create_publisher(
            PointStamped,
            "/target_pos",
            10)
        self.target_pos_subscriber = self.create_subscription(
            PointStamped, 
            "/target_pos",
            self.sub_callback,
            10)
        
        # self.my_socket = MySocketClass(recv_port, send_port, my_ip, other_ip)

    def sub_callback(self, msg:PointStamped):
        # subscribe target position from other selected robots
        # print(msg.point.x, msg.point.y)
        if not self.laser_detected and self.state == States.SELECTED and msg.header.frame_id != self.name:
            self.target_pos_x = -(msg.point.x - self.offset[msg.header.frame_id][0] + self.offset[self.name][0])
            self.target_pos_y = -(msg.point.y - self.offset[msg.header.frame_id][1] + self.offset[self.name][1])
            self.laser_detected = True
            print("receive target position")
            time.sleep(5.0)

    def setup_camera(self):
        print("start setup camera")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        self.config.enable_stream(rs.stream.depth, self.cam_width, self.cam_height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.cam_width, self.cam_height, rs.format.bgr8, 30)

        profile = self.pipeline.start(self.config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)

        print("Camera setup succeed.")

    def init_navigator(self):
        # Set nav2 initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)
    
    def handle_idle(self):
        while self.state == States.IDLE:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            red_bin = self.red_tracker.detect(color_image)
            green_bin = self.green_tracker.detect(color_image)

            if self.red_tracker.check_activate():
                print("red activate")
                self.state = States.SELECTED
                self.task = Tasks.SPIN
                self.cur_color = "red"

                self.change_state()
                break

            if self.green_tracker.check_activate():
                print("green activate")
                self.state = States.SELECTED
                self.cur_color = "green"
                self.change_state()
            
            break

    def handle_selected(self):
        while self.state == States.SELECTED:
            if self.laser_detected:
                self.state = States.NAVIGATING
                break

            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # images = np.hstack((color_image, depth_colormap))
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            red_bin = self.red_tracker.detect(color_image)
            red_pos = self.red_tracker.get_target_pos(depth_image, self.depth_scale, depth_intrin)
            print(red_pos)
            if self.cur_color == "red" and red_pos:
                self.laser_detected = True
                self.state = States.NAVIGATING
                self.target_pos_x = np.cos(self.cur_pose[2])*red_pos[0]\
                    - np.sin(self.cur_pose[2])*red_pos[1] + self.cur_pose[0]
                self.target_pos_y = np.sin(self.cur_pose[2])*red_pos[0]\
                    + np.cos(self.cur_pose[2])*red_pos[1] + self.cur_pose[1]
                self.rob_frame_target_position = red_pos
                pub_goal = PointStamped()
                pub_goal.header.frame_id = self.name
                pub_goal.point.x = self.target_pos_x
                pub_goal.point.y = self.target_pos_y
                self.target_pos_publisher.publish(pub_goal)
                print("publish")
                self.change_state()
                break

            green_bin = self.green_tracker.detect(color_image)
            green_pos = self.green_tracker.get_target_pos(depth_image, self.depth_scale, depth_intrin)
            print(green_pos)

            if self.cur_color == "green" and green_pos:
                self.laser_detected = True
                self.state = States.NAVIGATING
                # self.target_pos_x = green_pos[0]
                self.target_pos_x = np.cos(self.cur_pose[2])*green_pos[0]\
                    - np.sin(self.cur_pose[2])*green_pos[1] + self.cur_pose[0]
                # self.target_pos_y = green_pos[1]
                self.target_pos_y = np.sin(self.cur_pose[2])*green_pos[0]\
                    + np.cos(self.cur_pose[2])*green_pos[1] + self.cur_pose[1]
                self.rob_frame_target_position = green_pos
                pub_goal = PointStamped()
                pub_goal.header.frame_id = self.name
                pub_goal.point.x = self.target_pos_x
                pub_goal.point.y = self.target_pos_y
                self.target_pos_publisher.publish(pub_goal)
                print("publish")
                self.change_state()

                break
                # dat = str(self.target_pos_x) + " " + str(self.target_pos_y)
                # self.my_socket.send_data(dat)
            break

    def nav(self, px, py, oz):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = px #+ 0.08 * np.cos(self.cur_pose[2])
        goal_pose.pose.position.y = py #+ 0.08 * np.sin(self.cur_pose[2])
        print("target yaw: ", oz)
        goal_pose.pose.orientation.z = oz
        goal_pose.pose.orientation.w = 1.0

        # publish goal pose in this robot's world frame
        # if self.rob_frame_target_position:
        #     print("publishing goal position")
        #     pub_goal = PointStamped()
        #     pub_goal.point.x = px
        #     pub_goal.point.y = py
        #     pub_goal.header.frame_id='ff1'
        #     # for _ in range(10):
        #     self.target_pos_publisher.publish(pub_goal)
        spin_angle=min(abs(oz-self.cur_pose[2]),2*np.pi-abs(oz-self.cur_pose[2]))
        if spin_angle>0.5:
            self.navigator.spin((oz-self.cur_pose[2]+np.pi)%(2%np.pi)-np.pi)
            while not self.navigator.isTaskComplete():
                continue
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=15.0):
                self.navigator.cancelTask()
                self.state = States.FAILED
                return
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            # TODO: listen from tf2_ros transform_listener
            # self.tf_buffer.wait_for_transform_async()
            try: 
                self.on_tf_timer() 
                # self.cur_pose = [t.transform.translation.x, 
                #                 t.transform.translation.y, 
                #                 t.transform.rotation.z]
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')
            else:
                # print("lookup_transform failed")
                pass
            self.cur_pose = [self.cur_x, self.cur_y, self.cur_yaw]
            print("current pose: ", self.cur_pose)
            return 1
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        self.state = States.FAILED
        return 0

    def handle_navigate(self):
        while self.state == States.NAVIGATING:
            if self.rob_frame_target_position:
                target_orientation = self.cur_pose[2] + \
                    np.arctan2(self.rob_frame_target_position[1], self.rob_frame_target_position[0])                            
                # if self.nav(self.target_pos_x, self.target_pos_y, target_orientation):
                #     self.state = States.ARRIVE
            else:
                print("nav pose received from communication")
                # TODO: check correctness
                target_orientation = np.arctan2(self.target_pos_y - self.cur_pose[1], 
                                                self.target_pos_x - self.cur_pose[0])
                
            if self.nav(self.target_pos_x, self.target_pos_y, target_orientation):
                self.state = States.ARRIVE
                self.change_state()
                
            self.laser_detected = False
            break

    def handle_arrive(self):
        # TODO: control the bot to do something
        while self.state == States.ARRIVE:
            if not self.task:
                self.state = States.IDLE
                self.change_state()
                break

            if self.task == Tasks.SPIN:
                self.navigator.spin()
                self.navigator.spin()
                self.cur_pose[2] += 3.14
                # if self.nav(self.cur_pose[0], self.cur_pose[1], self.cur_pose[2]):
                #     pass

                print("complete spin task")
            elif self.task == Tasks.DRIVE_SQUARE:
                pass
            self.task = None
            self.state = States.IDLE
            self.cur_color = None
            self.rob_frame_target_position = None
            break

    def handle_failed(self):
        """reset pose when failed"""
        try:   
            self.on_tf_timer() 
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')
        else:
            # print("lookup transform failed")
            pass
        self.cur_pose = [self.cur_x, self.cur_y, self.cur_yaw]
        
        print("current pose: ", self.cur_pose)
        self.state = States.IDLE
        self.task = None
        self.cur_color = None
        self.rob_frame_target_position = None
        self.change_state()

    def quaternion_to_yaw(self,x, y, z, w):
        """
        Convert a quaternion into yaw angle.

        Parameters:
        x, y, z, w: Components of the quaternion.

        Returns:
        Yaw angle in radians.
        """
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def on_tf_timer(self):
        t = self.tf_buffer.lookup_transform(self.to_frame_rel,
                                            self.from_frame_rel,
                                            rclpy.time.Time(nanoseconds=0))
        self.cur_x = t.transform.translation.x
        self.cur_y = t.transform.translation.y
        self.cur_yaw = self.quaternion_to_yaw(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
        self.cur_timestamp = t.header.stamp
        print("transform: ", self.cur_x, " ", self.cur_y, " ", self.cur_yaw, " ",
               self.cur_timestamp)
    
    def change_state(self):
        if self.cur_pose[2] >= 6.28:
            self.cur_pose[2] -= 6.28
        if self.cur_pose[2] < 0:
            self.cur_pose[2] += 6.28
        self.red_tracker.clear_trail()
        self.green_tracker.clear_trail()
        print("changing state")
        time.sleep(1)

    def on_timer(self):
        # while True:
        if self.state == States.IDLE:
            print("idel")
            self.handle_idle()
        elif self.state == States.SELECTED:
            print("selected")
            self.handle_selected()
        elif self.state == States.NAVIGATING:
            print("navigating")
            self.handle_navigate()
        elif self.state == States.ARRIVE:
            print("arrived    may perform some tasks...")
            self.handle_arrive()
        elif self.state == States.FAILED:
            print("failed")
            self.handle_failed()

def main():
    """run with 'python3 runner.py -d' to show video"""
    rclpy.init()
    # parser = argparse.ArgumentParser(description='Run the Laser Tracker')
    # parser.add_argument('-d', '--display',
    #                     action='store_true',
    #                     help='Display the video or not')
    # params = parser.parse_args()

    # runner = Runner(params.display)
    runner = Runner(5001, 5005, "35.3.33.37", "35.3.202.217")
    # runner.start()
    try:
        rclpy.spin(runner)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
