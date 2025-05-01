import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float32
from cv_bridge import CvBridge, CvBridgeError
from skimage import morphology

class LaneBananaDetector():
    def __init__(self):
        self.bridge = CvBridge()
        
        # Image subscribers
        self.sub_image = rospy.Subscriber('/D435I/color/image_raw', Image, self.img_callback, queue_size=1)
        
        # Publishers
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.pub_err = rospy.Publisher("lane_detection/error", Float32, queue_size=1)
        self.pub_banana = rospy.Publisher("banana_detection/image", Image, queue_size=1)
        
        # Load banana classifier
        self.banana_cascade = cv2.CascadeClassifier('bananaclassifier.xml')
        
        # Lane detection parameters
        self.lane_line = Line(n=5)
        self.detected = False
        self.hist = True

    def detect_bananas(self, img):
        """Detect bananas in the image and return annotated image and banana count"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        bananas = self.banana_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        banana_count = 0
        annotated_img = img.copy()
        
        for (x, y, w, h) in bananas:
            banana_count += 1
            # Draw rectangle around the banana
            cv2.rectangle(annotated_img, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.putText(annotated_img, 'Banana', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        return annotated_img, banana_count

    def gradient_thresh(self, img, thresh_min=25, thresh_max=256):
        """Apply sobel edge detection on input image"""
        grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(grayscale, (5, 5), 0)
        sobelx = cv2.Sobel(blur_gray, cv2.CV_64F, 1, 0)
        sobely = cv2.Sobel(blur_gray, cv2.CV_64F, 0, 1)
        combine = cv2.addWeighted(sobelx, 0.7, sobely, 0.3, 0)
        abs_sobel = np.absolute(combine)
        scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel)) 
        binary_output = np.where((scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max), 1, 0)
        return binary_output

    def color_thresh(self, img, thresh=(100, 255)):
        """Convert RGB to HSL and threshold to binary image using S channel"""
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)   
        s_channel = hls[:, :, 2]
        h_channel = hls[:, :, 0]
        binary_output = np.where((h_channel < 70) & ((s_channel > thresh[0])), 1, 0)
        return binary_output

    def combinedBinaryImage(self, img):
        """Get combined binary image from color and sobel filters"""
        sobel = self.gradient_thresh(img)
        color = self.color_thresh(img)
        binaryImage = np.zeros_like(sobel)
        binaryImage[(color==1)] = 1
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'), min_size=50, connectivity=2)
        return binaryImage

    def perspective_transform(self, img):
        """Get bird's eye view from input image"""
        src = np.float32([[100,400], [639, 400], [639, 479], [50, 479]])
        dest = np.float32([[0,0], [639, 0], [639, 479], [0, 479]])
        M = cv2.getPerspectiveTransform(src, dest)
        Minv = np.linalg.inv(M)
        warped_img = cv2.warpPerspective(np.uint8(img), M, (640, 480))
        warped_img = cv2.dilate(warped_img, np.ones((3, 3), np.uint8), iterations=10)
        return warped_img, M, Minv

    def detection(self, img):
        """Main lane detection pipeline"""
        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)
        err = 0

        if not self.hist:
            ret = line_fit(img_birdeye)
            if ret is not None:
                lane_fit = ret['lane_fit']
                err = ret['err']
        else:
            if not self.detected:
                ret = line_fit(img_birdeye)
                if ret is not None:
                    lane_fit = ret['lane_fit']
                    err = ret['err']
                    lane_fit = self.lane_line.add_fit(lane_fit)
                    self.detected = True
            else:
                prev_fit = self.lane_line.get_fit()
                ret = tune_fit(img_birdeye, prev_fit)
                if ret is not None:
                    lane_fit = ret['lane_fit']
                    err = ret['err']
                    lane_fit = self.lane_line.add_fit(lane_fit)
                else:
                    self.detected = False

        bird_fit_img = None
        combine_fit_img = None
        if ret is not None:
            bird_fit_img = bird_fit(img_birdeye, ret)
            combine_fit_img = final_viz(img, lane_fit, Minv)
        else:
            rospy.logwarn("Unable to detect lane")

        return combine_fit_img, bird_fit_img, err

    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
            
        # Detect bananas
        banana_img, banana_count = self.detect_bananas(cv_image.copy())
        
        # Publish banana detection image
        try:
            self.pub_banana.publish(self.bridge.cv2_to_imgmsg(banana_img, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
        
        # Run lane detection
        lane_img, bird_img, err = self.detection(cv_image)
        
        if lane_img is not None and bird_img is not None:
            # Combine banana and lane annotations
            combined_img = cv2.addWeighted(lane_img, 0.7, banana_img, 0.3, 0)
            
            try:
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(combined_img, "bgr8"))
                self.pub_bird.publish(self.bridge.cv2_to_imgmsg(bird_img, "bgr8"))
                
                if err != 0.0:
                    if -100.0 < err < 100.0:
                        self.pub_err.publish(0)
                    else:
                        self.pub_err.publish(err / 100)
                else:
                    self.pub_err.publish(2000)
            except CvBridgeError as e:
                rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('lane_banana_detector')
    detector = LaneBananaDetector()
    rospy.spin()