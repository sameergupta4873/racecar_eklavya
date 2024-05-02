import cv2
import numpy as np

# Load YOLO
net = cv2.dnn.readNetFromDarknet("/home/sameergupta/catkin_ws/src/racecar_eklavya/controller/model/yolov3-tiny.cfg", "/home/sameergupta/catkin_ws/src/racecar_eklavya/controller/model/yolov3-tiny.weights")
classes = []
with open("/home/sameergupta/catkin_ws/src/racecar_eklavya/controller/model/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()

output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Function to get bounding box centroid
def get_centroid(x, y, w, h):
    centroid_x = x + (w / 2)
    centroid_y = y + (h / 2)
    return int(centroid_x), int(centroid_y)

# Initialize variables for tracking
prev_centroid = None
tracked_human = None

# Process live video
cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
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
            centroid = get_centroid(x, y, w, h)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
            cv2.putText(frame, 'Human', (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if prev_centroid is not None:
                distance = np.linalg.norm(np.array(centroid) - np.array(prev_centroid))
                if distance < min_distance:
                    min_distance = distance
                    closest_box = boxes[i]
            
    if len(boxes)!=0 and closest_box is None:
        closest_box=boxes[0]

    if closest_box is not None:
        x, y, w, h = closest_box
        centroid = get_centroid(x, y, w, h)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
        cv2.putText(frame, 'Human', (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        tracked_human = closest_box
        print(f"Human detected at Centroid : {centroid}")
        prev_centroid = centroid
    else:
        prev_centroid = None
        

    
    cv2.imshow("YOLO Object Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()