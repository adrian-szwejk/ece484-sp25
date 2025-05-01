#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # New message type for status
from cv_bridge import CvBridge

# X11: https://unix.stackexchange.com/questions/12755/how-to-forward-x-over-ssh-to-run-graphics-applications-remotely
class BananaDetector:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Load banana classifier
        self.banana_cascade = cv2.CascadeClassifier('bananaclassifier.xml')
        
        # Set up subscribers and publishers
        self.image_sub = rospy.Subscriber('/D435I/color/image_raw', Image, self.image_callback)
        self.detection_pub = rospy.Publisher('/banana_detections/image', Image, queue_size=1)
        self.status_pub = rospy.Publisher('/banana_detections/status', Bool, queue_size=1)  # New publisher
        
        rospy.loginfo("Banana Detector initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")
            return

        # Detect bananas
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        bananas = self.banana_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )

        # Publish detection status (True/False)
        detection_status = len(bananas) > 0
        self.status_pub.publish(detection_status)
        
        # Print human-readable status (optional)
        if detection_status:
            rospy.loginfo("Banana detected!")
        else:
            rospy.logdebug("No banana detected")

        # Draw detections on image
        for (x, y, w, h) in bananas:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.putText(cv_image, 'Banana', (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Publish annotated image
        try:
            self.detection_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except Exception as e:
            rospy.logerr(f"Image publishing failed: {e}")

if __name__ == '__main__':
    rospy.init_node('banana_detector')
    detector = BananaDetector()
    rospy.spin()