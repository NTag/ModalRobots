#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

class GoForward():
    def __init__(self):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_take = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.cmd_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        r = rospy.Rate(1)
        # Twist is a datatype for velocity
        move_take = Empty()
        rospy.sleep(5)
        self.cmd_take.publish(move_take)
        rospy.sleep(15)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_land.publish(Empty())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(3)
 
if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")

