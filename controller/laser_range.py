#!/usr/bin/env python3
import rospy	
from sensor_msgs.msg import LaserScan


def laserCallback(msg):
    boolean = 0
    obstacles = 0
    for i in range(len(msg.ranges)):
        if(msg.ranges[i] <= 10 and boolean ==0):
            obstacles+=1
            boolean = 1
        elif(boolean == 1 and msg.ranges[i]  > 10):
            boolean = 0
    	
    rospy.loginfo("No. of Obstacles : %f", obstacles)
    
        


def listener():
        rospy.init_node("listener",anonymous=True)
        rospy.Subscriber("/scan_multi", LaserScan , laserCallback)
        rospy.sleep(0.1)
        
if __name__ == '__main__':
        listener()
