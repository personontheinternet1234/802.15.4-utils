#!/usr/bin/env python

# Author: Isaac Verbrugge - isaacverbrugge@gmail.com
# Since: July 9, 2024
# Project: FOD Dog
# Purpose: bridge for ot -> waypointing

import rospy
import sys
import os 
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
import threading
from dog_door.openthread_bridge import OpenThreadClient


if(__name__ == "__main__"):
    try:
        rospy.init_node("openthread_bridge_node", anonymous=True)
        rospy.loginfo("IP: %s", os.getenv("IP"))
        wpan_ip = os.getenv("IP")

        # wpan_ip = "fd00:db8:a0:0:1cc7:5c8b:dd0d:3f0"

        if(wpan_ip == None):
            rospy.logerr("Missing wpan interface or wpan ip")

        dog_bridge_obj = OpenThreadClient(wpan_ip, port=5559)
        rospy.Subscriber('/fix', NavSatFix, dog_bridge_obj.update_known_gps)

        connected = dog_bridge_obj.connect()
        while(not connected):
            connected = dog_bridge_obj.connect()

        dog_bridge_obj.start_threads()

    except rospy.ROSInterruptException:
        sys.exit()
