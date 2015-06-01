#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class listener():
    def __init__(self):
        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/ardrone/front/image_raw', Image, self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        cv2.imshow("imw", cv_image)
        cv2.waitKey(25)

    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        rospy.sleep(3)
 
if __name__ == '__main__':
    ic = listener()
    rospy.init_node('ImageAnalyzer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
