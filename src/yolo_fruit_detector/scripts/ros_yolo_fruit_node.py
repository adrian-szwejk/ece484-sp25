#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2

# Globals
last_no_detection_time = None
NO_DETECTION_INTERVAL = rospy.Duration(2.0)

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', 
                       path='/home/nx/F1-Nineth/ece484-sp25/yolov5/best.pt', 
                       force_reload=False)
model.conf = 0.25  # Lowered to allow more detections
model.iou = 0.45

# Bridge to convert ROS images to OpenCV
bridge = CvBridge()

def callback(msg):
    global last_no_detection_time

    rospy.loginfo_once("Receiving image frames...")

    try:
        # Convert image to OpenCV format (BGR) then to RGB for YOLO
        img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        # Run inference
        results = model(img_rgb)
        detections = results.pandas().xyxy[0]

        if detections.empty:
            now = rospy.Time.now()
            if last_no_detection_time is None or (now - last_no_detection_time > NO_DETECTION_INTERVAL):
                rospy.loginfo("No fruit detected")
                last_no_detection_time = now
        else:
            for det in detections.itertuples():
                # Use the correct column names; print to verify once
                rospy.loginfo(f"Detected {det.name} at ({det.xmin:.0f},{det.ymin:.0f}) with conf {det.confidence:.2f}")

    except Exception as e:
        rospy.logerr(f"Detection error: {e}")

def main():
    rospy.init_node('yolo_fruit_detector')
    rospy.Subscriber('/D435I/color/image_raw', Image, callback, queue_size=1, buff_size=2**24)
    rospy.loginfo("YOLO fruit detector node started. Waiting for images...")
    rospy.spin()

if __name__ == '__main__':
    main()
