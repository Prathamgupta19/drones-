import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import torch
from models.experimental import attempt_load
from utils.dataloaders import LoadStreams, LoadImagesAndLabels
from utils.general import check_img_size, non_max_suppression
from utils.plots import save_one_box
from utils.torch_utils import select_device, time_sync

# Load the YOLOv5 model with pre-trained weights
weights = 'yolov5s.pt'  # Update with the path to your YOLOv5 model weights
device = select_device('')

model = attempt_load(weights, device=None)
imgsz = check_img_size(640, s=model.stride.max())  # Set image size

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert the NumPy array to a PyTorch float tensor
        image_tensor = torch.from_numpy(cv_image).to(device) / 255.0
        print(image_tensor.shape)
        # Add a batch dimension to the image
        image_tensor = image_tensor.unsqueeze(0)

        # Now, the image has shape [1, 480, 640, 3]
        print(image_tensor.shape)
        image_tensor = image_tensor.permute(0, 1, 2)  # Permute the dimensions from (H, W, C) to (C, H, W)

        # Perform object detection using YOLOv5
        results = model(image_tensor)  # Perform inference

        # Process the detection results
        for result in results.pred[0]:
            c = int(result[4])  # Class index
            conf = result[5]  # Confidence score
            if conf > 0.25:  # Adjust the confidence threshold as needed
                x1, y1, x2, y2 = map(int, result[:4])  # Bounding box coordinates
                label = f'Class {c}, Conf: {conf:.2f}'
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image with bounding boxes
        cv2.imshow("Object Detection", cv_image)
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

