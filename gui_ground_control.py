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
import gradio as gr
import requests
from PIL import Image
from io import BytesIO
import time
from util_classes import ServerClient, Waypoint

# Function to download the image and return it
def download_image():
    try:
        response = requests.get("<link>")
        img = Image.open(BytesIO(response.content))
        return img
    except:
        return None


class ThreadedServer(object):
    
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.clients = {}
        self.waypoints = {"Sim": Waypoint(39.9377789771,-91.194125972), "NIWC": Waypoint(21.379167, -157.966944), "Building 1": Waypoint(21.4297, -158.16050), "Tent (B81)": Waypoint(21.42989, -158.15516), "Close To Tent": Waypoint(21.42991, -158.15490), "Between": Waypoint(21.429902, -158.154332), "Intersection (N9)": Waypoint(21.429918, -158.153466), "Satcom": Waypoint(21.430089, -158.152998)}
        if not self.register_node():
            print("Lattice registration failure. Is this station connected to the internet?")
        print("Server ip: " + self.host)    
        threading.Thread(target=self.listen).start()
        threading.Thread(target=self.heartbeat).start()

        with gr.Blocks() as demo:
            timer = gr.Timer(0.1)

            title = gr.Markdown("# 5G Dog Ground Station")
            description = gr.Markdown("[*] Status: None")
            ip = gr.Markdown(f"IP={self.host}")

            with gr.Row():
                radio = gr.Radio(
                    ["relative", "coordinate", "stop", "waypoints"], label="Select a mode"
                )

                latitude = gr.Textbox(lines=1, visible=False, interactive=True)
                longitude = gr.Textbox(lines=1, visible=False, interactive=True)
                waypoint_choice = gr.Radio([], label="Select a preset waypoint", visible=False)

                radio.change(fn=self.mode, inputs=radio, outputs=(latitude, longitude, waypoint_choice))

            chosen_client_list = gr.CheckboxGroup(choices=[], label="Select Client(s)")

            send = gr.Button("Send Command")
            send.click(fn=self.send_command, inputs=(chosen_client_list, radio, latitude, longitude, waypoint_choice), outputs=description, api_name="Send Command")

            with gr.Row():
                output_image = gr.Image(width="50%", height="50%")
                map = gr.Image(value=Image.open("map.jpg"), width="50%", height="50%")

            timer.tick(self.show_possible_clients, outputs=chosen_client_list)
            timer.tick(download_image, outputs=output_image)

        demo.launch(share=False)


    def mode(self, choice):
        if choice == "relative":
            return (gr.Textbox(label="Relative Latitude (in 0.0001 degrees. 6.7 seems to be the maximum pathing will allow)", lines=1, visible=True, interactive=True, value=""), gr.Textbox(label="Relative longitude (in 0.0001 degrees. 6.7 seems to be the maximum pathing will allow)", lines=1, visible=True, interactive=True, value=""), gr.Radio([], label="Select a preset waypoint", visible=False))
        elif choice == "coordinate":
                return (gr.Textbox(label="Latitude", lines=1, visible=True, interactive=True, value=""), gr.Textbox(label="Longitude", lines=1, visible=True, interactive=True, value=""), gr.Radio([], label="Select a preset waypoint", visible=False))
        elif choice == "stop":
            return (gr.Textbox(label="Latitude", lines=1, visible=False, interactive=False, value=""), gr.Textbox(label="Longitude", lines=1, visible=False, interactive=False, value=""), gr.Radio([], label="Select a preset waypoint", visible=False))
        elif choice == "waypoints":
            return (gr.Textbox(label="Latitude", lines=1, visible=False, interactive=False, value=""), gr.Textbox(label="Longitude", lines=1, visible=False, interactive=False, value=""), gr.Radio(self.waypoints.keys(), label="Select a preset waypoint", visible=True, interactive=True))


    def show_possible_clients(self):
        
        if(len(self.clients) < 1):
            return gr.CheckboxGroup(label="No Clients Connected", choices=[])

        addr_list = []
        for address, client in self.clients.items():
            addr_list.append(address + " | Connection type: " + str(client.connection_type))

        return gr.CheckboxGroup(label="Select Client(s)", choices=addr_list)
    
    
    def heartbeat(self):
        while True:
            time.sleep(10)
            disconnected_clients = []
            for address, client in list(self.clients.items()):
                try:
                    client.sock_obj.sendall(b'HEARTBEAT')
                except Exception as e:
                    disconnected_clients.append(client)
            for client in disconnected_clients:
                self.close_client(client)
    

    def listen(self):
        self.server_socket.listen(0)
        while True:
            client_sock_obj, address = self.server_socket.accept()
            # client.settimeout(60)
            print("Client Connected: " + str(address[0]))

            client = ServerClient(client_sock_obj, address[0])

            self.clients[client.address] = client
            threading.Thread(target = self.listenToClient, args = (client, address)).start()


    def listenToClient(self, client, address):
        while True:
            try:
                data = client.sock_obj.recv(1024)
                # print(data)
                if data == b'':
                    self.close_client(client)
                    break

                if data:
                    try:
                        received_packet = json.loads(data)
                        if(received_packet.get("gps")):
                            self.publish_gps_to_lattice(received_packet["gps"]["lat"], received_packet["gps"]["lon"])
                            print("gps actual")
                        elif(received_packet.get("connection")): 
                            client.connection_type = received_packet["connection"]
                    except Exception as e:
                        print("Json error?" + str(e))
                    
            except Exception as e:
                print(e)
                break


    def close_client(self, client):
        print(f"Client Disconnected (Heartbeat): {client.address}")
        try:
            del self.clients[client.address]
            client.sock_obj.close()
        except Exception as e:
            print(e)
            pass    
            
            
    def send_command(self, chosen_client_list, mode, latitude, longitude, waypoint_choice):
        
        if(len(self.clients) > 0):

            for _ in chosen_client_list:
                chosen_address = _[:_.find("|") - 1]

                dictlist = []
                for key, value in self.clients.items():
                    temp = [key,value]
                    dictlist.append(temp)
    
                chosen_client = None
                if(self.clients.get(chosen_address)):
                    chosen_client = self.clients[chosen_address]
                else:
                    return f"<span style=\"color:red\">[-] Status: Can't send to specified client check if chosen client is still connected</span>."

                if(mode == "coordinate" or mode == "c"):
                    lat = latitude
                    lon = longitude

                    waypoint_json = "{\"waypoint\": {\"lat\": " + str(lat) + ",\"lon\": " + str(lon) + "}}"

                    chosen_client.sock_obj.send(waypoint_json.encode("utf-8"))

                elif(mode == "relative" or mode == "r"):
                    rel_lat = latitude
                    rel_lon = longitude
                    
                    rel_waypoint_json = "{\"relative_waypoint\": {\"rel_lat\": " + str(rel_lat) + ",\"rel_lon\": " + str(rel_lon) + "}}"

                    chosen_client.sock_obj.send(rel_waypoint_json.encode("utf-8"))

                elif(mode == "stop" or mode == "s"):

                    stop_json = "{\"stop\": 1}"

                    chosen_client.sock_obj.send(stop_json.encode("utf-8"))
                
                elif(mode == "waypoints" or mode == "w"): 
                    waypoint = self.waypoints[waypoint_choice]

                    waypoint_json = "{\"waypoint\": {\"lat\": " + str(waypoint.lat) + ",\"lon\": " + str(waypoint.lon) + "}}"

                    chosen_client.sock_obj.send(waypoint_json.encode("utf-8"))


                
            if(mode == "stop"):
                return f"<span style=\"color:red\">[*] Stopping client...</span>."
            elif(mode == "relative"):
                return f"<span style=\"color:green\">[+] Sending relative waypoint...</span>."
            elif(mode == "coordinate"):
                return f"<span style=\"color:green\">[+] Sending waypoint...</span>."

        else:
            return "Waiting for a client to connect..."    


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
        }

        url = 'https://ibp.anduril.com/v1/communicationsmanager/registerintegration'
        
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'Authorization': 'Bearer ' + <Token>
        }

        try:
            response = requests.put(url, json=registration_data, headers=headers)
            # print("Response from server: %s", response.text)
            
        except requests.exceptions.RequestException as e:
            print("Registration request failed: %s", e)
            return False
        
        return response.status_code == 200   

    
    def publish_gps_to_lattice(self, latitude, longitude):

        connection_type = "802.15.4"
        
        # Update times
        update_time = datetime.utcnow()
        expiry_time = update_time + timedelta(minutes=60)

        # Construct the payload
        lattice_entity = {
            "entity": {
                "isLive": True,
                "entityId": <ID>,
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
            'Authorization': 'Bearer ' + <Token>

        try:
            response = requests.post(url, json=lattice_entity, headers=headers)
            # print("Latitude: %s", str(latitude))
            # print("Longitude: %s", str(longitude))
            # print("Response from server: %s", response.text)
        except requests.exceptions.RequestException as e:
            print("Border Router / Ground Control not connected to internet", e)


if __name__ == "__main__":
    # my_ip = input("Ground Control's IP: ")
    # 21.42974
    # -158.16118

    my_ip = os.popen("sudo ot-ctl ipaddr mleid | head -n 1").read().strip()
    server = ThreadedServer(my_ip, 5559)
