#!/usr/bin/env python3
# BEGIN ALL
from distutils.log import error
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

error_area = 0
prev_err = 0
err_diff = 0
k_p = 4
k_i = 0
k_d = 0
correction = 0

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    image_sub = rospy.Subscriber('/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv =   cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = numpy.array([ 160, 50,  50])
    upper_red = numpy.array([180, 255, 255])
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
      print(f"Error : {err}    Error_Area : {error_area} Error_Diff : {err_diff}")
      print(correction)

      self.twist.linear.x = 3
      self.twist.angular.z = -float(correction) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
