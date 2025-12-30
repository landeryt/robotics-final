#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import zbar

class QRReader:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/qr/result", String, queue_size=1)
        rospy.Subscriber("/camera/image_raw", Image, self.image_cb)

        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')

    def image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        raw = gray.tobytes()

        image = zbar.Image(width, height, 'Y800', raw)
        self.scanner.scan(image)

        for symbol in image:
            data = symbol.data.decode("utf-8")
            rospy.loginfo(f"QR detected: {data}")
            self.pub.publish(data)
            break  # publish only first QR

if __name__ == "__main__":
    rospy.init_node("qr_reader")
    QRReader()
    rospy.spin()

