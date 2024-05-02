#!/usr/bin/env python3
from distutils.log import error
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import math

# Initialize MediaPipe hands module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Threshold for considering hand stillness (in pixels)
STILL_THRESHOLD = 10

# Threshold for finger lift detection (in pixels)
FINGER_LIFT_THRESHOLDS = [170, 200, 200, 200, 200]

# Initialize the video capture device
cap = cv2.VideoCapture(0)

direction = ['stop', 'left', 'forward', 'right', 'backward']+['stop']*7


def calculate_vector_length(point1, point2):
    """Calculate the length of a vector between two points"""
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)


class Teleop:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_robot', Twist, queue_size=1)
        self.twist = Twist()

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = cap.read()

            frame = cv2.flip(frame, 1)

            if not ret:
                break

            # Convert the frame to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Detect hands in the frame
            results = hands.process(rgb_frame)

            # Check if hands are detected
            if results.multi_hand_landmarks:
                # For simplicity, consider only the first detected hand
                hand_landmarks = results.multi_hand_landmarks[0]

                # Convert normalized coordinates to pixel coordinates
                h, w, _ = frame.shape
                hand_landmarks_px = [(int(lm.x * w), int(lm.y * h))
                                    for lm in hand_landmarks.landmark]

                # Calculate the palm center
                palm_center = hand_landmarks_px[0]

                # Calculate the tips of the fingers (landmarks 4, 8, 12, 16, 20)
                finger_tips = [hand_landmarks_px[i] for i in [4, 8, 12, 16, 20]]

                # Determine which fingers are lifted based on the length of the vectors from palm center to finger tips
                lifted_fingers = [1 if calculate_vector_length(
                    palm_center, tip) > FINGER_LIFT_THRESHOLDS[i] else 0 for i, tip in enumerate(finger_tips)]
                print([calculate_vector_length(palm_center, tip)
                    for tip in finger_tips])
                # Count the number of lifted fingers
                num_lifted_fingers = sum(lifted_fingers)

                # Display the number of lifted fingers
                cv2.putText(frame, f"Number of fingers lifted: {num_lifted_fingers - 1} - {direction[num_lifted_fingers-1]}", (
                    10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                print(direction[num_lifted_fingers-1])

                if(direction[num_lifted_fingers-1] == 'left'):
                    self.twist.linear.x = 1
                    self.twist.angular.z = 2
                elif(direction[num_lifted_fingers-1] == 'forward'):
                    self.twist.linear.x = 1
                    self.twist.angular.z = 0
                elif(direction[num_lifted_fingers-1] == 'right'):
                    self.twist.linear.x = 1
                    self.twist.angular.z = -2
                elif(direction[num_lifted_fingers-1] == 'backward'):
                    self.twist.linear.x = -1
                    self.twist.angular.z = 0
                else:
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0

            self.cmd_vel_pub.publish(self.twist)


            cv2.imshow('Hand Movement Detection', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def shutdown(self):
        # You can stop racecar by publishing an empty Twist message
        rospy.loginfo("Stopping Racecar")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.init_node('teleop')
        Teleop()
    except:
        rospy.loginfo("End of the trip for Racecar")
