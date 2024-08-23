#!/usr/bin/env python

# Author: James Clark - tahl.clark@gmail.com, Isaac Verbrugge - isaacverbrugge@gmail.com 
# Since: July 9, 2024 - Aug 21, 2024
# Project: FOD/5g Dog
# Purpose: Bridge for OT -> Waypointing

import rospy
import sys
import os
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
import threading
from dog_door.tak_client_bridge import TakClientBridge

#sys.path.append(os.path.abspath('../src'))
#from tak_client_bridge import TakClientBridge

if __name__ == "__main__":
    try:
        rospy.init_node("tak_client_bridge_node", anonymous=True)
        
        ipv4_tak_server = "192.168.8.165"
        rospy.loginfo("IP: %s", ipv4_tak_server)
        print("TAK Node Active")

        # rospy.loginfo("IP: %s", os.getenv("IP"))
        # wpan_ip = os.getenv("IP")
        # wpan_ip = "fd00:db8:a0:0:1cc7:5c8b:dd0d:3f0"

        # if ipv4_tak_server is None:
        #     rospy.logerr("Missing WPAN interface or WPAN IP")

        # TODO:
        # implement update_known_gps(), connect(), start_threads()

        dog_bridge_obj = TakClientBridge(ipv4_tak_server, port=8087)
        rospy.Subscriber('/fix', NavSatFix, dog_bridge_obj.update_known_gps)

        connected = dog_bridge_obj.connect()
        while not connected:
            connected = dog_bridge_obj.connect()

        dog_bridge_obj.start_threads()

    except rospy.ROSInterruptException:
        sys.exit()
