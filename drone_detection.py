import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.eval()

# Set confidence threshold
confidence_threshold = 0.25

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection using YOLOv5
        results = model(cv_image)

        # Process the detection results
        for result in results.pred[0]:
            c = int(result[5])  # Class index
            conf = result[4]  # Confidence score
            if conf > confidence_threshold:
                x1, y1, x2, y2 = map(int, result[:4])  # Bounding box coordinates
                label = f'Class {c}, Conf: {conf:.2f}'
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image with bounding boxes
        cv2.imshow("Camera Feed with Detection", cv_image)
        cv2.waitKey(1)

    except CvBridgeError as e:
        rospy.logerr(e)
        return

def camera_subscriber_publisher():
    rospy.init_node("camera_subscriber_publisher", anonymous=True)
    image_sub = rospy.Subscriber("/webcam/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        camera_subscriber_publisher()
    except rospy.ROSInterruptException:
        pass

