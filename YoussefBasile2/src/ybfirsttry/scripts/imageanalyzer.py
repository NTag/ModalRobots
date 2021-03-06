#!/usr/bin/env python

import numpy as np
import cv2
import video
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from common import anorm2, draw_str
from time import clock

lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

class listener():
    def __init__(self):
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.frame_idx = 0

        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/ardrone/front/image_raw', Image, self.callback)

    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        vis = frame.copy()
        height, width, depth = frame.shape

        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                #print "Point : {} {}".format(x,y)
                if len(tr) > self.track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
            self.tracks = new_tracks
            cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
            draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))

        if self.frame_idx % self.detect_interval == 0:
            #print " "
            i = 0
            vitesseg = 0
            vitessed = 0
            limit = width/3
            nbd = 0
            nbg = 0
            for tr in self.tracks:
                i = i + 1
                #print "Point {} : ".format(i),
                vitesse = tr[len(tr)-1][0] - tr[0][0]
                #for (x,y) in tr:
                #    print "({} {})".format(round(x,1),round(y,1)),
                #print format(round(vitesse,2))
                #print " "
                print "{} {} {} {}".format(i,round(tr[0][0],2),round(tr[0][1],2),round(vitesse,2))
                if tr[0][0] > width - limit:
                    vitessed = vitessed + vitesse
                    nbd = nbd + 1
                elif tr[0][0] < limit:
                    vitesseg = vitesseg + vitesse
                    nbg = nbg + 1
            if nbd == 0:
                nbd = 1
            if nbg == 0:
                nbg = 1
            vitessed = vitessed/nbd
            vitesseg = vitesseg/nbg
            #print "Vitesse gauche : {} ; Vitesse droite : {}".format(round(vitesseg,1),round(vitessed,1))
            mask = np.zeros_like(frame_gray)
            mask[:] = 255
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x, y), 5, 0, -1)
            p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y)])


        self.frame_idx += 1
        self.prev_gray = frame_gray
        cv2.imshow('lk_track', vis)
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
