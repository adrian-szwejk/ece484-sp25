import time
import math
import numpy as np
import cv2
import rospy
import PIL
import PIL.Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology

class LaneBananaDetector():
    def __init__(self):
        self.bridge = CvBridge()
        
        # Image subscribers
        self.sub_image = rospy.Subscriber('/D435I/color/image_raw', Image, self.img_callback, queue_size=1)
        
        # Vicon localization subscriber
        self.sub_vicon = rospy.Subscriber('/vicon/car_pose', PoseStamped, self.vicon_callback, queue_size=1)
        
        # Publishers
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.pub_err = rospy.Publisher("lane_detection/error", Float32, queue_size=1)
        self.pub_markers = rospy.Publisher("lane_detection/banana_markers", MarkerArray, queue_size=1)
        
        # Load banana classifier
        self.banana_cascade = cv2.CascadeClassifier('bananaclassifier.xml')
        
        # Single lane tracking
        self.lane_line = Line(n=5)
        self.detected = False
        self.hist = True
        
        # Track state
        self.current_pose = None
        self.banana_positions = []  # Stores global positions of detected bananas
        self.marker_id = 0
        
        # Camera parameters (calibrate these for your setup)
        self.camera_fov = 1.0472  # 60 degrees in radians
        self.camera_height = 0.2  # meters above ground
        self.camera_pitch = 0.35  # radians (about 20 degrees)

    def vicon_callback(self, msg):
        """Store the current Vicon pose for localization"""
        self.current_pose = msg.pose

    def detect_bananas(self, img):
        """Detect bananas in the image and return their positions relative to the car"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        bananas = self.banana_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        banana_positions = []
        
        for (x, y, w, h) in bananas:
            # Calculate position relative to car
            # First get position in camera coordinates
            img_center_x = img.shape[1] / 2
            img_center_y = img.shape[0] / 2
            
            # Calculate angle from center
            banana_center_x = x + w/2
            banana_center_y = y + h/2
            
            # Calculate horizontal angle (azimuth)
            horiz_angle = (banana_center_x - img_center_x) / img_center_x * (self.camera_fov/2)
            
            # Calculate distance (approximate based on size)
            # This is a rough estimate - you should calibrate this for your setup
            distance = 0.5 * img.shape[0] / h  # meters
            
            # Convert to car-relative coordinates
            x_rel = distance * math.sin(horiz_angle)
            y_rel = distance * math.cos(horiz_angle)
            
            banana_positions.append((x_rel, y_rel, distance))
            
            # Draw rectangle around the banana
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.putText(img, 'Banana', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        return img, banana_positions

    def update_banana_map(self, relative_positions):
        """Convert relative banana positions to global coordinates using Vicon"""
        if self.current_pose is None:
            return
            
        for rel_x, rel_y, distance in relative_positions:
            # Get car orientation from Vicon pose (quaternion to euler)
            q = self.current_pose.orientation
            yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
            
            # Transform to global coordinates
            global_x = self.current_pose.position.x + rel_x * math.cos(yaw) - rel_y * math.sin(yaw)
            global_y = self.current_pose.position.y + rel_x * math.sin(yaw) + rel_y * math.cos(yaw)
            
            # Check if this banana is already in our list (avoid duplicates)
            is_new = True
            for bx, by in self.banana_positions:
                if math.sqrt((global_x-bx)**2 + (global_y-by)**2) < 0.2:  # 20cm threshold
                    is_new = False
                    break
            
            if is_new:
                self.banana_positions.append((global_x, global_y))
                self.publish_banana_markers()

    def publish_banana_markers(self):
        """Publish markers for visualization in RViz"""
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(self.banana_positions):
            marker = Marker()
            marker.header.frame_id = "vicon_world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "bananas"
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(0)  # 0 means never auto-delete
            
            marker_array.markers.append(marker)
            self.marker_id += 1
        
        self.pub_markers.publish(marker_array)

    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        raw_img = cv_image.copy()
        
        # Detect bananas first
        banana_img, banana_positions = self.detect_bananas(raw_img.copy())
        self.update_banana_map(banana_positions)
        
        # Then do lane detection
        mask_image, bird_image, err = self.detection(raw_img)
        
        if mask_image is not None and bird_image is not None:
            # Combine banana and lane annotations
            combined_img = cv2.addWeighted(mask_image, 0.7, banana_img, 0.3, 0)
            
            out_img_msg = self.bridge.cv2_to_imgmsg(combined_img, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')
            
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)
            
            if err != 0.0:
                if -100.0 < err < 100.0:
                    self.pub_err.publish(0)
                else:
                    self.pub_err.publish(err / 100)
            else:
                self.pub_err.publish(2000)

    # ... (keep all the existing lane detection methods unchanged) ...

if __name__ == '__main__':
    rospy.init_node('lane_banana_detector', anonymous=True)
    detector = LaneBananaDetector()
    rospy.spin()