#!/usr/bin/env python

# Author: Isaac Verbrugge - isaacverbrugge@gmail.com
# Since: July 9, 2024
# Project: FOD Dog
# Purpose: Ground control server over 802.15.4 and wifi. handles lattice publishing and waypointing. 

import socket
import threading
import requests
import os
from datetime import datetime, timedelta
import json


class ThreadedServer(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.clients = {}
        if not self.register_node():
            print("Lattice registration failure. Is this station connected to the internet?")
        print("Server ip: " + self.host)    
        threading.Thread(target=self.listen).start()
        threading.Thread(target=self.prompt).start()
        

    def listen(self):
        self.server_socket.listen(0)
        while True:
            client, address = self.server_socket.accept()
            # client.settimeout(60)
            print("Client Connected: " + str(address[0]))
            self.clients[address[0]] = client
            threading.Thread(target = self.listenToClient, args = (client, address)).start()

    def listenToClient(self, client, address):
        size = 1024
        while True:
            try:
                data = client.recv(size)
                if data:
                    gps_packet = json.loads(data)
                    self.publish_gps_to_lattice(gps_packet["gps"]["lat"], gps_packet["gps"]["lon"], gps_packet["status"])
                    # print(gps_packet)
            except Exception as e:
                # print(e)
                pass
            
            
    def prompt(self):

        print("Waiting on connection...")
        
        while True:
            if(len(self.clients) > 0):


                dictlist = []
                for key, value in self.clients.items():
                    temp = [key,value]
                    dictlist.append(temp)

                print("\nPossible Clients: \n===========================")
                for i in range(len(dictlist)):
                    print(str(i) + ") " + dictlist[i][0])
                print("===========================\n")

                chosen_client = None
                while chosen_client == None:
                    inputted_client = input("Specify a client: ")
                    if(self.clients.get(inputted_client)):
                        chosen_client = self.clients[inputted_client]
                    else:
                        try:
                            chosen_client = dictlist[int(inputted_client)][1]
                        except:
                            pass


                mode = ""
                possible_modes = ["r","c","s","relative","coordinate", "stop"]
                while(mode not in possible_modes):
                    mode = input("Mode (relative, coordinate, STOP): ").lower()

                if(mode == "coordinate" or mode == "c"):
                    print("[Coordinate Mode]: Provide relative coordinates. Too far from the robot's current position and it will error out.")
                    lat = input("[Coordinate Mode] Provide lat: ")
                    lon = input("[Coordinate Mode] Provide lon: ")

                    waypoint_json = "{\"waypoint\": {\"lat\": " + str(lat) + ",\"lon\": " + str(lon) + "}}"

                    chosen_client.send(waypoint_json.encode("utf-8"))
                    print("Sending waypoint...")

                elif(mode == "relative" or mode == "r"):
                    print("[Relative Mode]: Provide relative coordinates (in 0.0001 degrees. 6.7 seems to be the maximum pathing will allow)")
                    rel_lat = input("[Relative Mode] Provide relative lat: ")
                    rel_lon = input("[Relative Mode] Provide relative lon : ")
                    
                    rel_waypoint_json = "{\"relative_waypoint\": {\"rel_lat\": " + str(rel_lat) + ",\"rel_lon\": " + str(rel_lon) + "}}"

                    chosen_client.send(rel_waypoint_json.encode("utf-8"))
                    print("Sending relative waypoint...")

                elif(mode == "stop" or mode == "s"):
                    print("[*] STOPPING... (not implemented yet)")
                    pass

    def register_node(self):
        registration_data = {
            "integrationName": "foddog",
            "details": {
                "entityDetails": {
                    "vendors": [
                        {
                            "vendorName": "turbineone",
                            "dataType": ["ads-b"]
                        }
                    ]
                }
            }
        }

        url = 'https://ibp.anduril.com/v1/communicationsmanager/registerintegration'
        
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'Authorization': 'Bearer ' + "TOKEN"        }

        try:
            response = requests.put(url, json=registration_data, headers=headers)
            # print("Response from server: %s", response.text)
            
        except requests.exceptions.RequestException as e:
            print("Registration request failed: %s", e)
            return False
        
        return response.status_code == 200   

    
    def publish_gps_to_lattice(self, latitude, longitude, connection_type):

        connection_type = "802.15.4"
        
        # Update times
        update_time = datetime.utcnow()
        expiry_time = update_time + timedelta(minutes=60)

        # Construct the payload
        lattice_entity = {
            "entity": {
                "isLive": True,
                "entityId": "ID",
                "expiryTime": expiry_time.isoformat() + 'Z',
                "aliases": {
                    "name": "5GDOG RANGER | Connection: " + connection_type
                },
                "provenance": {
                    "dataType": "ADSB",
                    "sourceUpdateTime": update_time.isoformat() + 'Z',
                },
                "ontology": {
                    "template": "TEMPLATE_TRACK"
                },
                "location": {
                    "position": {
                        "latitudeDegrees": latitude,
                        "longitudeDegrees": longitude,
                        "altitudeHaeMeters": 0,
                        "altitudeAglMeters": 0,
                    }
                },
                "milView": {
                    "disposition": "DISPOSITION_UNKNOWN",
                    "environment": "ENVIRONMENT_LAND",
                }
            }
        }

        # Send the POST request
        url = 'https://ibp.anduril.com/v1/entity/publish'
        
        headers = {
            'Accept': 'application/json',
            'Authorization': 'Bearer ' + "TOKEN"        }

        try:
            response = requests.post(url, json=lattice_entity, headers=headers)
            # print("Latitude: %s", str(latitude))
            # print("Longitude: %s", str(longitude))
            # print("Response from server: %s", response.text)
        except requests.exceptions.RequestException as e:
            print("Border Router / Ground Control not connected to internet", e)

if __name__ == "__main__":
    # my_ip = input("Ground Control's IP: ")
    my_ip = os.popen("sudo ot-ctl ipaddr mleid | head -n 1").read().strip()
    server = ThreadedServer(my_ip, 5559)
