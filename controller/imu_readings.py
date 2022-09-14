#!/usr/bin/env python3
import rospy	
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


roll = 0
pitch = 0
yaw = 0

def imuCallback(msg):
        orinetation_list = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        (roll,pitch,yaw) = euler_from_quaternion(orinetation_list)
        roll = roll * (180/3.14159265)
        pitch = pitch * (180/3.14159265)
        yaw = yaw * (180/3.14159265)
        rospy.loginfo("Yaw Angle --> %s", yaw)

def listener():
        rospy.init_node("listener",anonymous=True)
        rospy.Subscriber("/imu", Imu , imuCallback)
        rospy.spin()
if __name__ == '__main__':
        listener()
