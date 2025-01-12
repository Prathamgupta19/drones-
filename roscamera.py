#############DEPENDENCIES#######################


from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy

##############FUNCTIONS##########################



##Function to arm the drone props and takeoff at targetHeight (m)
def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Your image processing or other actions here
    # Example: Display the image using OpenCV
    cv2.imshow("Camera Feed", cv_image)
    cv2.waitKey(1)

def camera_subscriber_publisher():
    rospy.init_node("camera_subscriber_publisher", anonymous=True)
    image_sub = rospy.Subscriber("/webcam/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        camera_subscriber_publisher()
    except rospy.ROSInterruptException:
        pass


def arm_and_takeoff(targetHeight):

	while vehicle.is_armable!=True:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Look out! Virtual props are spinning!!")

	vehicle.simple_takeoff(targetHeight)

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")

	return None


############MAIN EXECUTABLE#############


####sim_vehicle.py opens up port on localhost:14550
print('thari')

vehicle = connect('127.0.0.1:14550',wait_ready=True)
print('mummy')

####Arm the drone and takeoff into the air at 5 meters


arm_and_takeoff(5)
print("Vehicle reached target altitude")


####Once drone reaches target altitude, change mode to LAND 



vehicle.mode=VehicleMode('LAND')
while vehicle.mode!='LAND':
	print("Waiting for drone to enter LAND mode")
	time.sleep(1)
print("Vehicle now in LAND mode. Will touch ground shortly.")
