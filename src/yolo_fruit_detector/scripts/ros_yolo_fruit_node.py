#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch

model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/YOUR_USERNAME/yolov5/best.pt', force_reload=False)
model.conf = 0.4
model.iou = 0.45

bridge = CvBridge()

def callback(msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = model(img)
        for det in results.pandas().xyxy[0].itertuples():
            rospy.loginfo(f"Detected {det.name} at ({det.xmin:.0f},{det.ymin:.0f}) with conf {det.confidence:.2f}")
    except Exception as e:
        rospy.logerr(f"Detection error: {e}")

def main():
    rospy.init_node('yolo_fruit_detector')
    rospy.Subscriber('/D435I/color/image_raw', Image, callback, queue_size=1, buff_size=2**24)
    rospy.spin()

if __name__ == '__main__':
    main()
