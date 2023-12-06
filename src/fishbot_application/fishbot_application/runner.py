# from laser_tracker import *
import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist, Pose, Point
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from enum import Enum

import sys
import argparse
import cv2
import numpy as np
import pyrealsense2 as rs
import time

class LaserTracker():
    def __init__(self, hsv_thres, color, cam_width, cam_height, xpos=0, ypos=0):
        self.cam_width = cam_width
        self.cam_height = cam_height
        self.xpos = xpos
        self.ypos = ypos

        self.tracker_height = 0.125
        self.horizontal_range = 69 * np.pi / 180

        self.hue_min = hsv_thres["hue_min"]
        self.hue_max = hsv_thres["hue_max"]
        self.sat_min = hsv_thres["sat_min"]
        self.sat_max = hsv_thres["sat_max"]
        self.val_min = hsv_thres["val_min"]
        self.val_max = hsv_thres["val_max"]
        self.color = color

        self.previous_position = None
        self.trail = np.zeros((self.cam_height, self.cam_width, 3),
                                 np.uint8)
        self.centers = [] # center[0]: width  center[1]: height
    
    def create_and_position_window(self, name, xpos, ypos):
        """Creates a named widow placing it on the screen at (xpos, ypos)."""
        # Create a window
        cv2.namedWindow(name)
        # Resize it to the size of the camera image
        cv2.resizeWindow(name, self.cam_width, self.cam_height)
        # Move to (xpos,ypos) on the screen
        cv2.moveWindow(name, xpos, ypos)

    def display(self, thres, frame):
        """Display the combined image and (optionally) all other image channels
        NOTE: default color space in OpenCV is BGR.
        """
        cv2.imshow("RGB_VideoFrame "+self.color, frame)
        cv2.imshow("LaserPointer "+self.color, thres)

    def setup_windows(self):
        sys.stdout.write("Using OpenCV version: {0}\n".format(cv2.__version__))

        # create output windows
        self.create_and_position_window("LaserPointer "+self.color, self.xpos, self.ypos)
        self.create_and_position_window("RGB_VideoFrame "+self.color,
                                        self.xpos + 10 + self.cam_width, self.ypos)

    def clear_trail(self):
        self.trail = np.zeros((self.cam_height, self.cam_width, 3), np.uint8)
        self.centers = []
        self.previous_position = None

    def track(self, frame, mask):
        """
        Track the position of the laser pointer.

        Code taken from
        http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        """
        center = None

        countours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)[-2]

        # only proceed if at least one contour was found
        if len(countours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(countours, key=cv2.contourArea)
            # c = min(countours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            moments = cv2.moments(c)
            if moments["m00"] > 0:
                center = int(moments["m10"] / moments["m00"]), \
                         int(moments["m01"] / moments["m00"])
            else:
                center = int(x), int(y)

            # only proceed if the radius meets a minimum size
            # if radius > 10:
            if radius > 10:
                # draw the circle and centroid on the frame,
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                # then update the ponter trail
                if self.previous_position:
                    cv2.line(self.trail, self.previous_position, center,
                             (255, 255, 255), 2)

        cv2.add(self.trail, frame, frame)
        self.previous_position = center
        if center:
            self.centers.append([center[0], center[1]])

    def find_blob(self, frame, mask):
        """Use blob detector to find laser point"""
        # Create the detector with the parameters
        detector = cv2.SimpleBlobDetector.create()
        params = detector.getParams()

        # Filter by color (white)
        params.filterByColor = False
        # params.blobColor = 255

        # Set Circularity filtering parameters 
        params.filterByCircularity = True 
        params.minCircularity = 0.6
        
        # Set Convexity filtering parameters 
        params.filterByConvexity = True
        params.minConvexity = 0.2

        # Set inertia filtering parameters 
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        # Filter by Area
        params.filterByArea = False
        params.maxArea = 200

        detector.setParams(params)
        
        # Detect blobs 
        keypoints = detector.detect(mask)
        
        print(len(keypoints))
        # if len(keypoints) and len(keypoints) <= 2:
            # print(keypoints)
            # print(len(keypoints))
        # max_y = 0
        center_x = 0
        center_y = 0
        for keypoint in keypoints:
            x, y = keypoint.pt  
            if y > center_y:
                center_y = y
                center_x = x

        if center_y > self.cam_height/3:
            center = int(center_x), int(center_y)
            print(center)
            if self.previous_position:
                cv2.line(self.trail, self.previous_position, center,
                                (255, 255, 255), 2)

            cv2.add(self.trail, frame, frame)
            self.previous_position = center
            self.centers.append([center[0], center[1]])

        print(self.centers)
        # if (len(self.centers) >= 25):
            # self.clear_trail()

    def detect(self, frame):
        # gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hls_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

        # gray_mask = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C
        #                                      , cv2.THRESH_BINARY, 5, 2)
        # gray_mask = cv2.threshold(gray_img, 200, 255, cv2.THRESH_BINARY)
        # blurred = cv2.GaussianBlur(gray_img, (11, 11), 0)
        # _, thres = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
        
        # Threshold ranges of HSV components
        # hsv_thres = cv2.inRange(hsv_img, (self.hue_min, self.sat_min, self.val_min), 
        #                             (self.hue_max, self.sat_max, self.val_max))
        # if self.color == "red":
        #     tmp = cv2.inRange(hsv_img, (170, self.sat_min, self.val_min),
        #                       (180, self.sat_max, self.val_max))
        #     hsv_thres = cv2.bitwise_or(hsv_thres, tmp)
        hls_thres = cv2.inRange(hls_img, (0, 250, 0), 
                                    (255, 255, 255))
        if self.color == "red":
            tmp = cv2.inRange(hls_img, (170, 200, self.sat_min),
                              (180, 255, self.sat_max))
            hls_thres = cv2.bitwise_or(hls_thres, tmp)
        
        # thres = cv2.bitwise_and(gray_mask, hsv_thres)
        # self.track(frame, hls_thres)
        self.find_blob(frame, hls_thres)
        # if self.previous_position:
            # print(hls_img[self.previous_position[1], self.previous_position[0]])

        # return hsv_thres
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
        if (len(self.centers) < 6):
            return False

        left = self.centers[0][0]
        right = self.centers[-1][0]
        if right > 500 and left < 140:
            return True
        return False
    
    def check_scattered_laser(self,mask):
        """Check the scattered laser points"""
        detector = cv2.SimpleBlobDetector.create()
        params = detector.getParams()

        # Filter by color (white)
        params.filterByColor = False
        # params.blobColor = 255

        # Set Circularity filtering parameters 
        params.filterByCircularity = True 
        params.minCircularity = 0.6
        
        # Set Convexity filtering parameters 
        params.filterByConvexity = True
        params.minConvexity = 0.2

        # Set inertia filtering parameters 
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        # Filter by Area
        params.filterByArea = False
        params.maxArea = 200

        detector.setParams(params)
        
        # Detect blobs 
        keypoints = detector.detect(mask)
        
        print(len(keypoints))
        if (len(keypoints) >= 5):
            return 1
        return 0
        # center_x = 0
        # center_y = 0
        # for keypoint in keypoints:

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

class Tasks(Enum):
    """enums for tasks"""
    SPIN = 1
    DRIVE_SQUARE = 2

class Runner():
    def __init__(self, display=False):
        self.display = display
        self.cam_width = 640
        self.cam_height = 480
        self.hsv_thres = {
            "red":{"hue_min":0, "hue_max":10, "sat_min":100, 
                   "sat_max":255, "val_min":200, "val_max":256}, 
            "green":{"hue_min":50, "hue_max":70, "sat_min":100, 
                   "sat_max":255, "val_min":200, "val_max":256}} # 100, 255
        self.red_tracker = LaserTracker(self.hsv_thres["red"], "red", self.cam_width, 
                                        self.cam_height)
        self.green_tracker = LaserTracker(self.hsv_thres["green"], "green", self.cam_width,
                                          self.cam_height, 0, self.cam_height+10)

        self.pipeline = None
        self.config = None
        self.align = None
        self.depth_scale = None
        # self.depth_intrin = None

        self.state = States.IDLE #States.IDLE
        self.task = None
        # self.offset = [0.0, 0.0]
        self.cur_pose = [0.0, 0.0, 0.0]
        self.target_pos_x = 0.0
        self.target_pos_y = 0.0
        self.laser_detected = False

        self.navigator = BasicNavigator()
        
        # self.target_pos = Point()
        # self.current_pos = Point()
        
        self.pub_node = Node("Robot1_publisher")
        self.sub_node = Node("Robot1_subscriber")
        self.target_pos_publisher = self.pub_node.create_publisher(
            PoseStamped,
            "Robot1_target_pos",
            10
        )
        self.target_pos_subscriber = self.sub_node.create_subscription(
            PoseStamped, 
            "Robot2_target_pos",
            self.sub_callback,
            10)

        # # self.pid = PID(1, 0.1, 0.05, setpoint=1)
        # self.pid = MYPID(1.0, 0.1, 0.03)

    def sub_callback(self, msg:PoseStamped):
        # subscribe target position from other selected robots
        if not self.laser_detected:
            self.target_pos_x = msg.pose.position.x
            self.target_pos_y = msg.pose.position.y

    def setup_camera(self):
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
        
    def handle_quit(self, delay=10):
        """Quit the program if the user presses "Esc" or "q"."""
        key = cv2.waitKey(delay)
        c = chr(key & 255)
        if c in ['c', 'C']:
            self.red_tracker.clear_trail()
            self.green_tracker.clear_trail()

        if c in ['q', 'Q', chr(27)]:
            cv2.destroyAllWindows()
            self.pipeline.stop()
            sys.exit(0)
    
    def handle_idle(self):
        while self.state == States.IDLE:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            # red_bin = self.red_tracker.detect(color_image)
            green_bin = self.green_tracker.detect(color_image)
            if self.display:
                # self.red_tracker.display(red_bin, color_image)
                self.green_tracker.display(green_bin, color_image)
                self.handle_quit()

            # if self.red_tracker.check_activate():
            if self.green_tracker.check_activate():
                self.state = States.SELECTED

    def handle_selected(self):
        while self.state == States.SELECTED:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))

            # red_bin = self.red_tracker.detect(color_image)
            green_bin = self.green_tracker.detect(color_image)
            # print(self.green_tracker.centers)
            # red_pos = self.red_tracker.get_target_pos(depth_image, self.depth_scale)
            # print(red_pos)
            # if red_pos:
                # self.state = States.NAVIGATING
                # TODO: control ...
            
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            green_pos = self.green_tracker.get_target_pos(depth_image, self.depth_scale, depth_intrin)
            print(green_pos)
            if self.display:
                # self.red_tracker.display(red_bin, images)
                self.green_tracker.display(green_bin, images)
                self.handle_quit()

            if (green_pos):
                self.laser_detected = True
                self.state = States.NAVIGATING
                # TODO: transform the target pos to world frame
                # self.target_pos_x = green_pos[0] + self.cur_pose[0]
                self.target_pos_x = green_pos[0]*np.cos(self.cur_pose[2]) \
                    - green_pos[1]*np.sin(self.cur_pose[2]) + self.cur_pose[0]
                # self.target_pos_y = green_pos[1] + self.cur_pose[1]
                self.target_pos_y = green_pos[0]*np.sin(self.cur_pose[2]) \
                    + green_pos[1]*np.cos(self.cur_pose[2]) + self.cur_pose[1]
                    
    def control_motion(self, px, py, oz=0.0):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = px
        goal_pose.pose.position.y = py
        goal_pose.pose.orientation.z = oz
        goal_pose.pose.orientation.w = 1.0
        
        # publish goal pose in this robot's world frame
        for _ in range(10):
            self.target_pos_publisher.publish(goal_pose)
            self.pub_node.get_logger().info("publishing goal pose")
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            continue
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            self.cur_pose = [self.target_pos_x, self.target_pos_y, 0.0]
            return 1
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        return 0

    def handle_navigate(self):
        while self.state == States.NAVIGATING:
            if self.control_motion(self.target_pos_x, self.target_pos_y):
                self.state = States.ARRIVE
                break
            self.laser_detected = False
            # frames = self.pipeline.wait_for_frames()
            # aligned_frames = self.align.process(frames)
            # depth_frame = aligned_frames.get_depth_frame()
            # color_frame = frames.get_color_frame()

            # if not depth_frame or not color_frame:
            #     continue

            # depth_image = np.asanyarray(depth_frame.get_data())
            # color_image = np.asanyarray(color_frame.get_data())
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # images = np.hstack((color_image, depth_colormap))

            # # red_bin = self.red_tracker.detect(color_image)
            # green_bin = self.green_tracker.detect(color_image)
            # # print(self.green_tracker.centers)
            # # red_pos = self.red_tracker.get_target_pos(depth_image, self.depth_scale)
            # # print(red_pos)
            # # if red_pos:
            #     # self.state = States.NAVIGATING
            #     # TODO: control ...
            
            # depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            # green_pos = self.green_tracker.get_target_pos(depth_image, self.depth_scale, depth_intrin)
            # print(green_pos)
            # if self.display:
            #     # self.red_tracker.display(red_bin, images)
            #     self.green_tracker.display(green_bin, images)
            #     self.handle_quit()

            # if (green_pos):
            #     self.target_pos = green_pos

    def handle_arrive(self):
        # TODO: control the bot to do something
        while self.state == States.ARRIVE:
            start = time.time()
            while (start+5.0) > time.time():
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())

                green_bin = self.green_tracker.detect(color_image)
                if self.display:
                    self.green_tracker.display(green_bin, color_image)
                    self.handle_quit()
                
                # if self.green_tracker.check_scattered_laser(green_bin):
                    # self.task = Tasks.SPIN

            if not self.task:
                self.state = States.IDLE
                break

            if self.task == Tasks.SPIN:
                if self.control_motion(self.target_pos_x, self.target_pos_y, 1.0):
                    pass
                if self.control_motion(self.target_pos_x, self.target_pos_y, 1.0):
                    pass
                print("complete spin task")
            elif self.task == Tasks.DRIVE_SQUARE:
                pass
            self.task = None
            self.state = States.IDLE
            
    def start(self):
        rclpy.spin(self.pub_node)
        rclpy.spin(self.sub_node)

        if self.display:
            print("Display enabled")
        if self.display:
            # self.red_tracker.setup_windows()
            self.green_tracker.setup_windows()
        self.setup_camera()
        # TODO: initialize current_pos
        # self.current_pos = [0, 0]

        # Set nav2 initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        # self.navigator.waitUntilNav2Active()
        print("wait for nav2")
        time.sleep(5.0)

        while True:
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

            self.red_tracker.clear_trail()
            self.green_tracker.clear_trail()
            print("changing state")
            time.sleep(1)

        scv2.destroyAllWindows()
        self.pipeline.stop()
        rclpy.shutdown()
        # navigator.lifecycleShutdown()
        sys.exit(0)

def main():
    """run with 'python3 runner.py -d' to show video"""
    rclpy.init()
    # parser = argparse.ArgumentParser(description='Run the Laser Tracker')
    # parser.add_argument('-d', '--display',
    #                     action='store_true',
    #                     help='Display the video or not')
    # params = parser.parse_args()

    # runner = Runner(params.display)
    runner = Runner()
    runner.start()
    