#!/usr/bin/env python3
# Execute as a python script  
# Set linear and angular values of TurtleBot's speed and turning. 
from distutils.log import error
import rospy				# Needed to create a ROS node
from geometry_msgs.msg import Twist     # Message that moves base
import sys 

class ControlRacecar():
    def __init__(self, lin_vel, ang_vel):
        # ControlRacecar is the name of the node sent to the master
        rospy.init_node('ControlRacecar', anonymous=False)

	# Message to screen
        rospy.loginfo(" Press CTRL+c to stop Racecar")

        # Keys CNTL + c will stop script   
        rospy.on_shutdown(self.shutdown)
        
	#Publisher will send Twist message on topic cmd_vel
       
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
     
	# Racecar will receive the message 10 times per second. 
        rate = rospy.Rate(10);   
	# 10 Hz is fine as long as the processing does not exceed 1/10 second.

        # Twist is a type of geometry_msgs for linear and angular velocity
        move_cmd = Twist()
	# Linear speed in x in meters/second is + (forward) or - (backwards)
        move_cmd.linear.x = lin_vel	# Modify this value to change speed
	# let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel	# Modify this value to cause rotation rad/s

	# Loop and TurtleBot will move until you type CNTL+c
        while not rospy.is_shutdown():
	    # publish the Twist values to the Racecar node /cmd_vel
            self.cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            rospy.loginfo("Linear value is --> %f , Angular value is --> %f", float(sys.argv[1]),float(sys.argv[2]))
            rate.sleep()
                        
        
    def shutdown(self):
        # You can stop racecar by publishing an empty Twist message 
        rospy.loginfo("Stopping Racecar")
	# 
        self.cmd_vel.publish(Twist())
	# Give Racecar time to stop
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        ControlRacecar(float(sys.argv[1]), float(sys.argv[2]))
    except:
        rospy.loginfo("End of the trip for Racecar")
