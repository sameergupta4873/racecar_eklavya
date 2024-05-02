#!/usr/bin/env python3
# BEGIN ALL
from distutils.log import error
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

error_area = 0
prev_err = 0
err_diff = 0
k_p = 2
k_i = 0
k_d = 4
correction = 0

# Load YOLO
net = cv2.dnn.readNetFromDarknet("/home/sameergupta/catkin_ws/src/racecar_eklavya/controller/model/yolov3-tiny.cfg",
                                 "/home/sameergupta/catkin_ws/src/racecar_eklavya/controller/model/yolov3-tiny.weights")
classes = []
with open("/home/sameergupta/catkin_ws/src/racecar_eklavya/controller/model/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()

output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Initialize variables for tracking
prev_centroid = None
tracked_human = None

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("window", 1)
        image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                                     Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_robot',
                                           Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        global prev_centroid, tracked_human
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, channels = frame.shape
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and class_id == 0:  # Class ID for human
                    # Object detected
                    center_x, center_y, w, h = (detection[0:4] * np.array([width, height, width, height])).astype(int)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        min_distance = float('inf')
        closest_box = None

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                centroid = self.get_centroid(x, y, w, h)

                if prev_centroid is not None:
                    distance = np.linalg.norm(np.array(centroid) - np.array(prev_centroid))
                    if distance < min_distance:
                        min_distance = distance
                        closest_box = boxes[i]
                
        if len(boxes)!=0 and closest_box is None:
            closest_box=boxes[0]

        if closest_box is not None:
            x, y, w, h = closest_box
            centroid = self.get_centroid(x, y, w, h)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
            cv2.putText(frame, 'Human', (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            tracked_human = closest_box
            prev_centroid = centroid
            # pid
            global prev_err, err_diff,error_area,k_p, k_i , k_d
            err = centroid[0] - width/2
            error_area += err
            err_diff = err - prev_err
            prev_err = err
            correction = k_p*err + k_d*err_diff + k_i*error_area
            print(f"Error : {err}    Error_Area : {error_area} Error_Diff : {err_diff}")
            print(correction)

            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(correction) / 100
            self.cmd_vel_pub.publish(self.twist)

        else:
            prev_centroid = None
            self.cmd_vel_pub.publish(Twist())
            

        
        cv2.imshow("YOLO Object Detection", frame)
        cv2.waitKey(3)

    # Function to get bounding box centroid


    def get_centroid(self, x, y, w, h):
        centroid_x = x + (w / 2)
        centroid_y = y + (h / 2)
        return int(centroid_x), int(centroid_y)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
