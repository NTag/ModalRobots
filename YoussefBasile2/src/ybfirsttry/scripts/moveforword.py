#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class GoForward():
    def __init__(self):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_reset = rospy.Publisher('/ardrone/reset', Empty, queue_size=1)
        self.cmd_take = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.cmd_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.sleep(5)	
        self.cmd_reset.publish(Empty())

        r = rospy.Rate(10)
        # Twist is a datatype for velocity
        move_take = Empty()
        self.cmd_take.publish(move_take)
        move_up = Twist()
        move_up.linear.z = 0.3
        self.cmd_vel.publish(move_up)
        self.cmd_vel.publish(move_up)
        self.cmd_vel.publish(move_up)
        rospy.sleep(1)
        self.cmd_vel.publish(move_up)
        while not rospy.is_shutdown():
            move_forward = Twist()
            move_forward.linear.x = 0.3
            move_forward.linear.z = 0
            move_forward.angular.z = 1
            self.cmd_vel.publish(move_forward)
            r.sleep()

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Drone")
        r = rospy.Rate(10)
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        move_forward = Twist()
        move_forward.linear.x = 1
        move_forward.linear.z = 0
        move_forward.angular.z = 0
        self.cmd_vel.publish(move_forward)
        r.sleep()
        self.cmd_vel.publish(move_forward)
        r.sleep()
        self.cmd_vel.publish(move_forward)
        r.sleep()
        self.cmd_vel.publish(move_forward)
        r.sleep()
        self.cmd_vel.publish(move_forward)
        r.sleep()
        self.cmd_vel.publish(move_forward)
        r.sleep()
        self.cmd_land.publish(Empty())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(3)
 
if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")

