#!/usr/bin/env python

# Author: Isaac Verbrugge - isaacverbrugge@gmail.com
# Since: July 9, 2024
# Project: FOD Dog
# Purpose: Bridge for TAK server -> waypointing

import socket
import threading
import json
import os
import rospy
from std_msgs.msg import Bool, String
from dog_door.srv import AchieveGPSWaypoint, AchieveGPSWaypointResponse
from dog_door.msg import GpsPoint

class TakClientBridge:
    def __init__(self, server_ip, port=8087):
        # Initialize connection parameters
        self.server_ip = server_ip
        self.port = port
        self.socket = None
        self.gps = None
        self.connection_type = ""

        # Initialize ROS publishers
        self.pub_latlong_target = rospy.Publisher("/robot_exec/current_wp_latlon", GpsPoint, queue_size=1)
        self.plan_status_pub = rospy.Publisher("/robot_exec/plan_status", String, queue_size=1)
        self.set_estop_pub = rospy.Publisher("/soft_estop/trigger", Bool, queue_size=1)
        self.reset_estop_pub = rospy.Publisher("/soft_estop/reset", Bool, queue_size=1)

        # Set up ROS rate objects
        self.rate = rospy.Rate(1)
        self.rate2 = rospy.Rate(1)

        print("TakClientBridge initialized with server {}:{}".format(server_ip, port))
        rospy.loginfo("TakClientBridge initialized with server %s:%s", server_ip, port)

    def update_known_gps(self, gps):
        """Update the known GPS location"""
        self.gps = gps
        # Uncomment for debugging
        # print("Updated GPS: Lat {}, Lon {}".format(self.gps.latitude, self.gps.longitude))
        # rospy.loginfo("Updated GPS: Lat %f, Lon %f", self.gps.latitude, self.gps.longitude)

    def connect(self):
        """Establish connection to the TAK server"""
        self.rate.sleep()
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_ip, self.port))
            print("[TAK_CLIENT_BRIDGE] Connected to server at {}:{}".format(self.server_ip, self.port))
            rospy.loginfo("[TAK_CLIENT_BRIDGE] Connected to server at %s:%s", self.server_ip, self.port)
            return True
        except Exception as e:
            print("[TAK_CLIENT_BRIDGE] Not Connected: {}".format(e))
            rospy.logerr("[TAK_CLIENT_BRIDGE] Not Connected: %s", e)
            return False

    def start_threads(self):
        """Start the listening and GPS sending threads"""
        threading.Thread(target=self.listen, name="ListenThread").start()
        threading.Thread(target=self.send_ot_gps, name="GPSSendThread").start()
        print("Started listen and GPS send threads")
        rospy.loginfo("Started listen and GPS send threads")

    def send_ot_gps(self):
        """Continuously send GPS data to the server"""
        while not rospy.is_shutdown():
            try:
                self.rate2.sleep()
                self.connection_type = "5G"  # Assuming 5G connection for this implementation

                if self.gps:
                    gps_data = {
                        "gps": {
                            "lat": self.gps.latitude,
                            "lon": self.gps.longitude
                        }
                    }
                    gps_packet = json.dumps(gps_data)
                    self.socket.sendall((gps_packet + '\n').encode("utf-8"))
                    print("[TAK_CLIENT_BRIDGE] Sent GPS data: {}".format(gps_packet))
                    rospy.loginfo("[TAK_CLIENT_BRIDGE] Sent GPS data: %s", gps_packet)

                    connection_data = {"connection": self.connection_type}
                    connection_packet = json.dumps(connection_data)
                    self.socket.sendall((connection_packet + '\n').encode("utf-8"))
                    print("[TAK_CLIENT_BRIDGE] Sent connection type: {}".format(connection_packet))
                    rospy.loginfo("[TAK_CLIENT_BRIDGE] Sent connection type: %s", connection_packet)

            except Exception as e:
                print("[TAK_CLIENT_BRIDGE] Error sending GPS data: {}".format(e))
                rospy.logerr("[TAK_CLIENT_BRIDGE] Error sending GPS data: %s", e)

    def listen(self):
        """Listen for incoming data from the server"""
        while not rospy.is_shutdown():
            try:
                data = self.socket.recv(1024).decode("utf-8")
                if data:
                    if data == "HEARTBEAT":
                        print("Received HEARTBEAT")
                        rospy.loginfo("Received HEARTBEAT")
                        continue

                    print("[TAK_CLIENT_BRIDGE] Received: {}".format(data))
                    rospy.loginfo("[TAK_CLIENT_BRIDGE] Received: %s", data)

                    tak_packet = json.loads(data)

                    if "waypoint" in tak_packet:
                        lat = tak_packet["waypoint"]["lat"]
                        lon = tak_packet["waypoint"]["lon"]
                        print("[TAK_CLIENT_BRIDGE] Received waypoint: lat {}, lon {}".format(lat, lon))
                        rospy.loginfo("[TAK_CLIENT_BRIDGE] Received waypoint: lat %f, lon %f", lat, lon)
                        threading.Thread(target=self.achieve, args=(lat, lon), name="AchieveThread").start()

                    elif "stop" in tak_packet:
                        print("[TAK_CLIENT_BRIDGE] Received STOP command")
                        rospy.loginfo("[TAK_CLIENT_BRIDGE] Received STOP command")
                        self.set_estop_pub.publish(True)
                        self.reset_estop_pub.publish(False)

                    else:
                        print("[TAK_CLIENT_BRIDGE] Received unknown data type")
                        rospy.logwarn("[TAK_CLIENT_BRIDGE] Received unknown data type")

            except json.JSONDecodeError as e:
                print("[TAK_CLIENT_BRIDGE] JSON decoding error: {}".format(e))
                rospy.logerr("[TAK_CLIENT_BRIDGE] JSON decoding error: %s", e)

            except Exception as e:
                print("[TAK_CLIENT_BRIDGE] Error in listen thread: {}".format(e))
                rospy.logerr("[TAK_CLIENT_BRIDGE] Error in listen thread: %s", e)
                self.connect()

    def achieve(self, lat, lon):
        """Send the robot to the specified waypoint"""
        rospy.wait_for_service("/foddog/achieve_gps_waypoint")
        waypointProxy = rospy.ServiceProxy("/foddog/achieve_gps_waypoint", AchieveGPSWaypoint)

        print("Sending robot to waypoint - Lat: {}, Lon: {}".format(lat, lon))
        rospy.loginfo("Sending robot to waypoint - Lat: %f, Lon: %f", lat, lon)

        g_point = GpsPoint()
        g_point.lat = lat
        g_point.lon = lon
        self.pub_latlong_target.publish(g_point)
        self.plan_status_pub.publish('executing')

        try:
            response = waypointProxy(g_point, GpsPoint(), Bool(True))
            print("Waypoint service response: {}".format(response))
            rospy.loginfo("Waypoint service response: %s", response)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
            rospy.logerr("Service call failed: %s", e)

    def is_connected_to_wifi(self, check_rate=5):
        """Check if connected to WiFi"""
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=check_rate)
            print("Connected to WiFi")
            rospy.loginfo("Connected to WiFi")
            return True
        except OSError:
            print("Not connected to WiFi")
            rospy.logwarn("Not connected to WiFi")
            pass
        except Exception as e:
            print("Error checking WiFi connection: {}".format(e))
            rospy.logerr("Error checking WiFi connection: %s", e)
            return False
        return False

if __name__ == "__main__":
    try:
        rospy.init_node('tak_client_bridge')
        print("Initialized ROS node: tak_client_bridge")
        rospy.loginfo("Initialized ROS node: tak_client_bridge")

        server_ip = rospy.get_param('~server_ip', '192.168.8.165')
        tak_bridge = TakClientBridge(server_ip=server_ip, port=8087)

        if tak_bridge.connect():
            tak_bridge.start_threads()
            rospy.spin()
        else:
            print("Failed to connect to the server. Exiting.")
            rospy.logerr("Failed to connect to the server. Exiting.")

    except rospy.ROSInterruptException:
        print("ROS node interrupted")
        rospy.loginfo("ROS node interrupted")

    except Exception as e:
        print("Unexpected error: {}".format(e))
        rospy.logerr("Unexpected error: %s", e)
        raise
