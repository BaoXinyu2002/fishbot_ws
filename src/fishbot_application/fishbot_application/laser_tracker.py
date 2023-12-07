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