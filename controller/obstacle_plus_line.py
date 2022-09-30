#!/usr/bin/env python3
from math import atan2 ,tan ,exp
from statistics import mean
from turtle import pu
import numpy as np
import rospy ,cv2, cv_bridge	
from sensor_msgs.msg import LaserScan , Imu , Image , CameraInfo
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

#repulsive constant values
robot_width = 1.5 # 145mm + 160mm + 145mm = 450mm = 0.45 m
max_range = 10
gamma = 5
theta_goal = 0
#imu
roll = 0
pitch = 0
yaw = 0
#move
move = Twist()
pub = rospy.Publisher("/cmd_vel" , Twist)

def imuCallback(msg):
        orinetation_list = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        (roll,pitch,yaw) = euler_from_quaternion(orinetation_list)
        roll = roll * (180/3.14159265)
        pitch = pitch * (180/3.14159265)
        yaw = yaw * (180/3.14159265)


def calculate_repulsive_feild(laser_readings, start,end):
    # variables for repulsive feild:-
    #calculating average distance of obstacle -> d_k:-
    d_k = mean(laser_readings[start:end+1])
    #calculating  angle occupied by obstacle -> fi_k:-
    # as for 542 readings of lidar ==> pi radians of area is covered for 1 reading ==> 0.0058 radians :-
    fi_k = 0.0058 * (end - start)
    # as we now have d_k and fi_k we can calculate -> sigma_k :-
    global robot_width # obstacle enlargening 
    sigma_k = atan2( (d_k * tan(fi_k/2)) + robot_width /2 , d_k )
    # calculating A_k :-
    global max_range
    d_k_dash = max_range - d_k
    A_k = d_k_dash * exp(- 1 / 2)
    #calculating center angle -> theta_k :-
    theta_k = (((start+end) / 2 ) * 0.0058) - 1.57079632679
    # calculating repulsive feild for all the laser reading of the given obstacle
    repulsive_feild = []
    for theta_i in range(542) :
        repulsive_feild_function = A_k * ( exp (- (((theta_k - ((theta_i*0.0058)-1.57079632679))**2)/(2*(sigma_k**2)))))
        repulsive_feild.append(repulsive_feild_function)
    return repulsive_feild



def calculate_attractive_feild(theta_goal):
    global gamma
    attractive_feild = []
    for theta_i in range(542) :
        attractive_feild_function = (gamma)*abs(theta_goal - ((theta_i*0.0058)-1.57079632679))
        attractive_feild.append(attractive_feild_function)
    return attractive_feild


    


def obstacle_avoidance(msg ):
    # final feilds
    total_net_feild = np.zeros(542)
    total_repulsive_feild = np.zeros(542)
    total_attractive_feild = np.zeros(542)
    #laser reading
    laser_readings = msg


    #calculating repulsive feild
    start = 0
    end = -1
    obstacle_endpoints = set()
    for i in range(start,len(laser_readings)-1):
        if( laser_readings[i]  > 2 and laser_readings[i+1] <=2):
            start = i+1
        elif(laser_readings[i]  <= 2 and laser_readings[i+1] > 2 ):
            end = i
        if(start <= end and end != -1):
            obstacle_endpoints.add((start,end))
    #print(obstacle_endpoints)
    #giving laser readings and obstacle endpoints
    for elements in obstacle_endpoints :
        total_repulsive_feild= np.add(total_repulsive_feild,calculate_repulsive_feild(laser_readings, elements[0],elements[1]))
    
    #calculating attractive field
    global yaw
    global theta_goal
    total_attractive_feild = calculate_attractive_feild(theta_goal)

    # finally total net feild 
    total_net_feild = np.add(total_attractive_feild,total_repulsive_feild)
    # minima angle for which net feild is minimum
    
    min_value_index  = np.where(total_net_feild == np.amin(total_net_feild))
    angle_for_min_radians = (0.0058*(min_value_index[0][0]) - 1.57079632679)
    angle_for_min_degrees = ((angle_for_min_radians*(180)) / 3.14159265359)
    
    # publishing spin to achieve the angle_for_min 
    
    
    global move
    move.angular.z = 0
    print(angle_for_min_degrees)
    
    if(abs(angle_for_min_degrees - yaw ) > 0.5):
        if(angle_for_min_degrees > yaw ):
            move.angular.z = 6
        elif( yaw  > angle_for_min_degrees ):
            move.angular.z = -6
    else:
        move.angular.z = 0
        move.linear.x = 0.5
    global pub
    pub.publish(move)


######## Line Follower ########

error_area = 0
prev_err = 0
err_diff = 0
k_p = 3
k_i = 0
k_d = 0
correction = 0


def line_follower():
    #cv2.namedWindow("window", 1)
    image_sub = rospy.Subscriber('/camera/rgb/image_raw', 
                                      Image, image_callback)
    
    
def image_callback( msg):
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv =   cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([ 160, 50,  50])
    upper_red = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    h, w, d = image.shape
    search_top = 3*h//4
    search_bot = 3*h//4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      global prev_err, err_diff,error_area,k_p, k_i , k_d
      err = cx - w/2
      error_area += err
      err_diff = err - prev_err
      prev_err = err
      correction = k_p*err + k_d*err_diff + k_i*error_area
      global move
      move.linear.x = 0.3
      move.angular.z = -float(correction) / 100
      global pub
      pub.publish(move)
      # END CONTROL
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)



def laserCallback(msg):
    laser_readings = msg.ranges
    count = 0
    for i in range(245, 300):
        if(laser_readings[i] == 11.0):
            count+=1
    if(count < 45):
        obstacle_avoidance(laser_readings)
    else:
        line_follower()
    
        
def listener():
    rospy.init_node("listener",anonymous=True)
    rospy.Subscriber("/imu", Imu , imuCallback)
    rospy.Subscriber("/scan_multi", LaserScan , laserCallback)
    # rospy.Subscriber('/camera/rgb/image_raw', Image, imageCallback)
    rospy.spin()
        
if __name__ == "__main__":
    listener()

