#!/usr/bin/env python

# Author: Isaac Verbrugge - isaacverbrugge@gmail.com, Kayla Uyema - kaylauyema@gmail.com
# Since: August 5, 2024
# Project: FOD Dog
# Purpose: bridge for ot -> waypointing

import socket
import threading
import json
import os
import rospy
import sys
from std_msgs.msg import Bool
from dog_door.srv import AchieveGPSWaypoint, AchieveGPSWaypointResponse
from dog_door.msg import GpsPoint
from std_msgs.msg import String

class NIWCClient:
    
    def __init__(self, server_ip, openthread_port=5559, tak_port=8087):
        self.server_ip = server_ip
        self.openthread_port = openthread_port
        self.tak_port = tak_port
        self.openthread_socket = None
        self.tak_socket = None
        self.client_address = None
        self.gps = None
        self.connection_type = ""
        
        self.pub_latlong_target = rospy.Publisher("/robot_exec/current_wp_latlon", GpsPoint, queue_size=1)
        self.plan_status_pub = rospy.Publisher("/robot_exec/plan_status", String, queue_size=1)
        self.set_estop_pub = rospy.Publisher("/soft_estop/trigger", Bool, queue_size=1)
        self.reset_estop_pub = rospy.Publisher("/soft_estop/reset", Bool, queue_size=1)
        self.rate = rospy.Rate(1)
        self.rate2 = rospy.Rate(1)

    def update_known_gps(self, gps):
        self.gps = gps

    def connect_openthread(self):
        self.rate.sleep()
        try:
            self.openthread_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            self.openthread_socket.connect((self.server_ip, self.openthread_port))
             
            print("[NIWC_BRIDGE] Connected OpenThread node!")
        except Exception:
            print("[NIWC_BRIDGE] OpenThread not Connected")
            return False
        return True

    def connect_tak(self):
        self.rate.sleep()
        try:
            self.tak_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tak_socket.connect((self.server_ip, self.tak_port))
             
            print("[NIWC_BRIDGE] Connected TAK node!")
        except Exception:
            print("[NIWC_BRIDGE] TAK not Connected")
            return False
        return True

    def send_ot_gps(self):
        while True:
            try:
                self.rate2.sleep()
                ifconfig_result = str(os.popen("ifconfig").read())
                self.connection_type = "NO CONNECTION"
                if("wpan0" in ifconfig_result):
                    self.connection_type = "802.15.4"
                if(self.is_connected_to_wifi()):
                    self.connection_type = "5G"

                if(self.connection_type == "802.15.4" and self.gps != None): 
                    gps_packet = "{\"gps\": {\"lat\": " + str(self.gps.latitude) + ",\"lon\": " + str(self.gps.longitude) + "}}"
                    self.openthread_socket.sendall(gps_packet.encode("utf-8"))
                    
                connection_packet = "{\"connection\": \"" + str(self.connection_type) + "\"}"
                self.openthread_socket.sendall(connection_packet.encode("utf-8"))

                # could send gps to tak server here
                self.tak_socket.sendall(connection_packet.encode("utf-8"))

            except:
                pass        
    
    def start_threads(self):
        threading.Thread(target=self.listen_openthread).start()
        threading.Thread(target=self.listen_tak).start()
        threading.Thread(target=self.send_ot_gps).start()

    def listen_openthread(self):
        while True:
            try:
                openthread_data = self.openthread_socket.recv(1024)
                
                if(openthread_data != ""):
                    
                    if(openthread_data == "HEARTBEAT"):
                        continue

                    print("[OPENTHREAD_BRIDGE] Received: {}".format(openthread_data))

                    ot_packet = json.loads(openthread_data)

                    if(ot_packet.get("waypoint")):
                        lat = ot_packet["waypoint"]["lat"]
                        lon = ot_packet["waypoint"]["lon"]

                        # if(self.data != None):
                        #     print("[OPENTHREAD_BRIDGE] Current pos is lat: {}, long: {}".format(self.gps.latitude, self.gps.longitude))
                        # else:
                        #     print("[OPENTHREAD_BRIDGE] Current /fix data is None")    

                        print("[OPENTHREAD_BRIDGE] Attempting to go to waypoint lat: {}, long: {} once done waiting for /foddog/achieve_gps_waypoint service".format(lat, lon))

                        threading.Thread(target=self.achieve, args=(lat, lon)).start()

                    elif(ot_packet.get("relative_waypoint")):
                        rel_lat = ot_packet["relative_waypoint"]["rel_lat"]
                        rel_lon = ot_packet["relative_waypoint"]["rel_lon"]

                        if(self.gps != None):
                            lat = self.gps.latitude + 0.0001 * rel_lat
                            lon = self.gps.longitude + 0.0001 * rel_lon
                            print("[OPENTHREAD_BRIDGE] Current pos is lat: {}, long: {}".format(self.gps.latitude, self.gps.longitude))
                        else:
                            lat = 0
                            lon = 0
                            print("[OPENTHREAD_BRIDGE] Current /fix data is None")  

                        print("[OPENTHREAD_BRIDGE] Attempting to go to waypoint lat: {}, long: {} once done waiting for /foddog/achieve_gps_waypoint service".format(lat, lon))

                        threading.Thread(target=self.achieve, args=(lat,lon)).start()

                    elif(ot_packet.get("stop")):
                        print("[OPENTHREAD_BRIDGE] STOPPING...")
                        self.set_estop_pub.publish(True)
                        self.reset_estop_pub.publish(False)

                else:
                    self.connect_openthread()
            except:
                pass
    
    def listen_tak(self):
        while True:
            try:
                
                tak_data = self.tak_socket.recv(1024)

                if(tak_data != ""):
                    
                    if(tak_data == "HEARTBEAT"):
                        continue

                    print("[TAK_BRIDGE] Received: {}".format(tak_data))

                    ot_packet = json.loads(tak_data)

                    if(ot_packet.get("waypoint")):
                        lat = ot_packet["waypoint"]["lat"]
                        lon = ot_packet["waypoint"]["lon"]

                        print("[TAK_BRIDGE] Attempting to go to waypoint lat: {}, long: {} once done waiting for /foddog/achieve_gps_waypoint service".format(lat, lon))

                        threading.Thread(target=self.achieve, args=(lat, lon)).start()

                    elif(ot_packet.get("relative_waypoint")):
                        rel_lat = ot_packet["relative_waypoint"]["rel_lat"]
                        rel_lon = ot_packet["relative_waypoint"]["rel_lon"]

                        if(self.gps != None):
                            lat = self.gps.latitude + 0.0001 * rel_lat
                            lon = self.gps.longitude + 0.0001 * rel_lon
                            print("[TAK_BRIDGE] Current pos is lat: {}, long: {}".format(self.gps.latitude, self.gps.longitude))
                        else:
                            lat = 0
                            lon = 0
                            print("[TAK_BRIDGE] Current /fix data is None")  

                        print("[TAK_BRIDGE] Attempting to go to waypoint lat: {}, long: {} once done waiting for /foddog/achieve_gps_waypoint service".format(lat, lon))

                        threading.Thread(target=self.achieve, args=(lat,lon)).start()

                    elif(ot_packet.get("stop")):
                        print("[TAK_BRIDGE] STOPPING...")
                        self.set_estop_pub.publish(True)
                        self.reset_estop_pub.publish(False)
                else:
                    self.connect_tak()
            
            except:
                pass
                
    def achieve(self, lat, lon):
        rospy.wait_for_service("/foddog/achieve_gps_waypoint")
        waypointProxy = rospy.ServiceProxy("/foddog/achieve_gps_waypoint", AchieveGPSWaypoint)

        print(lat)
        print(lon)

        g_point = GpsPoint()
        g_point.lat = lat
        g_point.lon = lon
        self.pub_latlong_target.publish(g_point)
        self.plan_status_pub.publish('executing')
        waypointProxy(g_point, GpsPoint(), Bool(True))       

    def is_connected_to_wifi(self, check_rate=5):
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=check_rate)
            return True
        except OSError:
            pass
        except Exception:
            return False
        return False                

if __name__ == "__main__":
    server_ip = input("Server's ip: ") 
    NIWCClient(server_ip=server_ip,openthread_port=5559,tak_port=8087)
    
