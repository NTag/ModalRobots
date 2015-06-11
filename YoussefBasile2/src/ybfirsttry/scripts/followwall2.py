#!/usr/bin/env python

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata
import numpy as np
import cv2
import video
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from common import anorm2, draw_str
from time import clock
import math

help_message = '''
Fait decoller le drone,
puis fait lui suivre le mur
a sa droite.

'''
class FollowWall():
    def draw_flow(self, img, flow, step=16):
        h, w = img.shape[:2]
        y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
        fx, fy = flow[y,x].T
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.polylines(vis, lines, 0, (0, 255, 0))
        for (x1, y1), (x2, y2) in lines:
            cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
        return vis

    def callback(self,data):
        try:
            frameh = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        if not self.fly:
            return

        # On reduit fortement la resolution de l'image
        # et on ne s'interesse qu'au tiers droit.
        h, w = frameh.shape[:2]
        frame = frameh[0:h:4, (2*w/3):(w):4]
        h, w = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.first:
            self.first = False
            self.prevgray = gray
        else:
            height, width, depth = frame.shape

            flow = cv2.calcOpticalFlowFarneback(self.prevgray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            self.prevgray = gray

            i = 0
            vitessed = 0
            nbd = 0
            emoyen = 0
            k = 0
            for m in flow:
                k += 1
                for (a,b) in m:
                    if (a > 1 or a < -1):
                        vitesse = a
                        nbd = nbd + 1
                        # Comme l'image a ete redimensionnee,
                        # il faut recalculer les coordonnees
                        # initiales du point
                        vitessedt = self.vitesset(2*w/3 + (k%w)*4, math.floor(k/w))
                        #print "{} {} >> {} ({} %)".format(vitesse, vitessedt, vitesse - vitessedt, 100*(vitesse - vitessedt)/vitessedt)
                        emoyen = emoyen + 100*(vitesse - vitessedt)/vitessedt

            if nbd == 0:
                nbd = 1
                emoyen = -100
            vitessed = vitessed/nbd
            emoyen = emoyen/nbd
            print "Ecart moyen : {}".format(emoyen)
            if nbd >= 20:
                if emoyen >= 20 || emoyen <= -20:
                    self.speed[self.speed['c']] = emoyen
                    self.speed['c'] += 1 % 3
                    if not self.speed['finish'] && self.speed['c'] == 0:
                        self.speed['finish'] = True
                    if self.speed['finish']:
                        emoyen3 = (self.speed[0] + self.speed[1] + self.speed[2])/3
                        print "Ecart moyenne 3 : {}".format(emoyen3)
                        self.vx = math.max(0, 0.1 - math.abs(emoyen3)/1000)
                        self.az = emoyen3/100

            cv2.imshow('flow', self.draw_flow(gray, flow))
            cv2.waitKey(25)

    def __init__(self):
        self.hasInfos = False
        self.fly = False
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/ardrone/front/image_raw', Image, self.callback)
        self.infoss = rospy.Subscriber('/ardrone/navdata', Navdata, self.verifystatus)
        self.first = True
        rospy.init_node('FollowWall', anonymous=False)
        rospy.loginfo("To stop AR.Drone CTRL + C")
        rospy.on_shutdown(self.shutdown)

        self.cmd_reset = rospy.Publisher('/ardrone/reset', Empty, queue_size=1)
        self.cmd_take = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.cmd_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        while not rospy.is_shutdown():
            r = rospy.Rate(10)
            r.sleep()

    def vitesset(self, x, y):
        return 0.5*(2.28127677e+01 -2.72574113e-01*x -1.49826090e-02*y +7.83386484e-04*x*x +3.24478877e-04*x*y -2.27923981e-04*y*y)

    def verifystatus(self, data):
        if not self.hasInfos:
            print "Batterie : {}%".format(data.batteryPercent)
            print "Statut : ", data.state
            if data.state == 0:
                self.hasInfos = True
                rospy.sleep(5)
                rospy.loginfo("Initialisation")
                self.cmd_reset.publish(Empty())
                rospy.sleep(5)
                rospy.loginfo("Decollage");
                self.takeoff()
            elif data.state == 2:
                self.hasInfos = True
                rospy.loginfo("Decollage")
                self.takeoff()
            rospy.sleep(1)

    def takeoff(self):
        rospy.sleep(5)
        move_take = Empty()
        self.cmd_take.publish(move_take)
        self.vx = 0.1
        self.az = 0
        self.nbp = 0
        self.nbm = 0
        self.speed = {0: 0, 1: 0, 2: 0, 'c': 0, 'finish': False}
        r = rospy.Rate(10)
        self.fly = True
        while not rospy.is_shutdown():
            move_forward = Twist()
            move_forward.linear.x = self.vx
            move_forward.linear.y = 0
            move_forward.linear.z = 0
            move_forward.angular.z = self.az
            self.cmd_vel.publish(move_forward)
            r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop Drone")
        self.fly = False
        r = rospy.Rate(10)
        self.cmd_land.publish(Empty())
        rospy.sleep(3)

if __name__ == '__main__':
    try:
        FollowWall()
    except:
        rospy.loginfo("FollowWall node terminated.")
