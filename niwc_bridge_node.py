#!/usr/bin/env python

# Author: Isaac Verbrugge - isaacverbrugge@gmail.com, Kayla Uyema - kaylauyema@gmail.com
# Since: August 5, 2024
# Project: FOD Dog
# Purpose: bridge for ot -> waypointing

import rospy
import sys
import os 
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
import threading
from dog_door.niwc_bridge import NIWCClient


if(__name__ == "__main__"):
    try:
        rospy.init_node("niwc_bridge_node", anonymous=True)
        rospy.loginfo("IP: %s", os.getenv("IP"))
        wpan_ip = os.getenv("IP")

        # wpan_ip = "fd00:db8:a0:0:1cc7:5c8b:dd0d:3f0"

        if(wpan_ip == None):
            rospy.logerr("Missing wpan interface or wpan ip")

        dog_bridge_obj = NIWCClient(wpan_ip, openthread_port=5559, tak_port=8087)
        rospy.Subscriber('/fix', NavSatFix, dog_bridge_obj.update_known_gps)

        openthread_connected = dog_bridge_obj.connect_openthread()
        tak_connected = dog_bridge_obj.connect_tak()

        while (not openthread_connected):
            print("Retrying OpenThread connection...")
            openthread_connected = dog_bridge_obj.connect_openthread()
        
        if (not tak_connected):
            print("TAK not connected")

        dog_bridge_obj.start_threads()

    except rospy.ROSInterruptException:
        sys.exit()
