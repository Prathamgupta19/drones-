#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not overload the FCU with messages.
    rate = rospy.Rate(3)

    # Specify your waypoints here
    goals = [
        [0, 0, 3, 0],  # Waypoint 1: x=0, y=0, z=3, psi=0
        [5, 0, 3, -90],  # Waypoint 2: x=5, y=0, z=3, psi=-90
        [5, 5, 3, 0],  # Waypoint 3: x=5, y=5, z=3, psi=0
        [0, 5, 3, 90],  # Waypoint 4: x=0, y=5, z=3, psi=90
    ]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
    # Land after all waypoints are reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached, landing now." + CEND)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()

