import asyncio
from datetime import datetime
import json
from pprint import pprint
from math import pi
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import pickle
import struct

import arena
from drive_instrux import instrux_list
from geom2d import pt_coords
from robot_ble_connection import BleConnection

# Offset + sensor value = actual distance to robot center
OFFSETS = [19, 24, 15, 15, 15, 10, 20]

# Saved data file
data_file = "saved_data.pkl"

instrux_gen = (drive_dict for drive_dict in instrux_list)  # generator

class RobotDisplay:
    def __init__(self):
        self.ble_connection = BleConnection(self.handle_data)
        self.buffer = ""
        self.arena = {"arena": arena.boundary_lines,}
        self.wp_list = []
        self.waypoints = None
        self.closed = False
        self.fig, self.axes = plt.subplots()
        self.pose_list = []
        self.poses = None
        self.r_pnts_list = []
        self.r_pnts = None
        self.r30_pnts_list = []
        self.r30_pnts = None
        self.r60_pnts_list = []
        self.r60_pnts = None
        self.l_pnts_list = []
        self.l_pnts = None
        self.l30_pnts_list = []
        self.l30_pnts = None
        self.l60_pnts_list = []
        self.l60_pnts = None
        self.f_pnts_list = []
        self.f_pnts = None
        self.fwd_pnts_list = []
        self.fwd_pnts = None
        self.robot_is_ready = True

    def handle_close(self, _):
        self.closed = True

    def handle_data(self, data):
        self.buffer += data.decode()
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            try:
                # Print current time w/ seconds (to hundredths)
                now = datetime.now()
                current_time = now.strftime("%H:%M:%S.%f")[:-4]
                print(current_time)

                # Print data from robot
                message = json.loads(line)
                pprint(message)
            except ValueError:
                print("Error parsing JSON")
                return
            if "status" in message:
                if message["status"] == "READY":
                    self.robot_is_ready = True
                    try:
                        self.drive(1)
                        print("Starting next drive instruction")
                    except StopIteration:
                        print("No more drive instructions")
            if "pose" in message:
                pose = message["pose"]
                self.pose_list.append(pose)
                self.poses = np.array(self.pose_list, dtype=np.float32)
            if "distances" in message:
                distances = message["distances"]
                # Process distance data from VCSEL sensors
                corrected_dists = []
                pt_locations = []  # global coords of detected points
                for idx in range(len(distances)):
                    if distances[idx] < 1500:
                        corrected_value = distances[idx] + OFFSETS[idx]
                        corrected_dists.append(corrected_value)
                        rel_angle = pi/2 - idx * pi/6
                        xy_coords = pt_coords(pose, corrected_value/1000, rel_angle)
                        pt_locations.append(xy_coords)
                        # Sensor A (Left looking)
                        if idx == 0:
                            self.l_pnts_list.append(xy_coords)
                            self.l_pnts = np.array(self.l_pnts_list, dtype=np.float32)
                        # Sensor B (Left 30 deg fwd looking)
                        if idx == 1:
                            self.l30_pnts_list.append(xy_coords)
                            self.l30_pnts = np.array(self.l30_pnts_list, dtype=np.float32)
                        # Sensor C (Left 60 deg fwd looking)
                        if idx == 2:
                            self.l60_pnts_list.append(xy_coords)
                            self.l60_pnts = np.array(self.l60_pnts_list, dtype=np.float32)
                        # Sensor D (forward looking)
                        elif idx == 3:
                            self.f_pnts_list.append(xy_coords)
                            self.f_pnts = np.array(self.f_pnts_list, dtype=np.float32)
                        # Sensor E (Right 60 deg fwd looking)
                        if idx == 4:
                            self.r60_pnts_list.append(xy_coords)
                            self.r60_pnts = np.array(self.r60_pnts_list, dtype=np.float32)
                        # Sensor F (Right 30 deg fwd looking)
                        if idx == 5:
                            self.r30_pnts_list.append(xy_coords)
                            self.r30_pnts = np.array(self.r30_pnts_list, dtype=np.float32)
                        # Sensor G (right looking)
                        elif idx == 6:
                            self.r_pnts_list.append(xy_coords)
                            self.r_pnts = np.array(self.r_pnts_list, dtype=np.float32)


    def draw(self):
        self.axes.clear()
        if self.arena:
            for line in self.arena["arena"]:
                self.axes.plot(
                    [line[0][0], line[1][0]], [line[0][1], line[1][1]], color="black"
                )
        if self.waypoints is not None:
            self.axes.scatter(self.waypoints[:,0], self.waypoints[:,1], color="magenta")
        if self.poses is not None:
            self.axes.scatter(self.poses[:,0], self.poses[:,1], color="blue")
        if self.r_pnts is not None:
            self.axes.scatter(self.r_pnts[:,0], self.r_pnts[:,1], color="darkgreen")
        if self.r30_pnts is not None:
            self.axes.scatter(self.r30_pnts[:,0], self.r30_pnts[:,1], color="lime")
        if self.r60_pnts is not None:
            self.axes.scatter(self.r60_pnts[:,0], self.r60_pnts[:,1], color="yellowgreen")
        if self.l_pnts is not None:
            self.axes.scatter(self.l_pnts[:,0], self.l_pnts[:,1], color="darkred")
        if self.l30_pnts is not None:
            self.axes.scatter(self.l30_pnts[:,0], self.l30_pnts[:,1], color="crimson")
        if self.l60_pnts is not None:
            self.axes.scatter(self.l60_pnts[:,0], self.l60_pnts[:,1], color="pink")
        if self.f_pnts is not None:
            self.axes.scatter(self.f_pnts[:,0], self.f_pnts[:,1], color="yellow")
        
    async def send_waypoint(self, point):
        if self.robot_is_ready:
            wp_req = "!DWP".encode('utf8') + (json.dumps(point) + "\n").encode()
            print(f"Sending waypoint to robot: {wp_req}")
            await self.ble_connection.send_uart_data(wp_req)
            self.robot_is_ready = False

    async def send_turn_gh(self, hdg):
        if self.robot_is_ready:
            reqst = "!TGH".encode('utf8') + struct.pack('f', hdg) + ("\n").encode()
            print(f"Sending Goal Heading to robot: {reqst}")
            await self.ble_connection.send_uart_data(reqst)
            self.robot_is_ready = False

    async def send_instruct(self, ):
        if self.robot_is_ready:
            try:
                instruction = next(instrux_gen)
                print(instruction)
                (cmd, val), = instruction.items()
                if "WP" in cmd:
                    reqst = cmd.encode('utf8') + (json.dumps(val) + "\n").encode('utf8')
                else:
                    reqst = cmd.encode('utf8') + struct.pack('f', val) + ("\n").encode('utf8')
                print(f"Sending Drive Instruction to robot: {reqst}")
                await self.ble_connection.send_uart_data(reqst)
                self.robot_is_ready = False
            except StopIteration:
                print("No more Driving Instructions")

    async def send_command(self, code):
        request = (code + "\n").encode('utf8')
        print(f"Sending request: {request}")
        await self.ble_connection.send_uart_data(request)

    def drive(self, _):
        self.button_task = asyncio.create_task(self.send_instruct())

    def wapo(self, _):
        self.button_task = asyncio.create_task(self.send_waypoint(next(waypogen)))

    def turn(self, _):
        self.button_task = asyncio.create_task(self.send_turn_gh(next(turngen)[0]))

    def run(self, _):
        self.button_task = asyncio.create_task(self.send_command("!RUN"))

    def stop(self, _):
        self.button_task = asyncio.create_task(self.send_command("!STP"))

    def save_data(self, ):
        data = {"poses": self.poses,
                "r_pnts": self.r_pnts,
                "r30_pnts": self.r30_pnts,
                "r60_pnts": self.r60_pnts,
                "l_pnts": self.l_pnts,
                "l30_pnts": self.l30_pnts,
                "l60_pnts": self.l60_pnts,
                "f_pnts": self.f_pnts,
                }
        with open(data_file, 'wb') as file:
            pickle.dump(data, file)

    async def main(self):
        plt.ion()
        await self.ble_connection.connect()
        try:
            self.fig.canvas.mpl_connect("close_event", self.handle_close)
            drive_button = Button(plt.axes([0.2, 0.85, 0.1, 0.075]), "Drive")
            drive_button.on_clicked(self.drive)
            wapo_button = Button(plt.axes([0.4, 0.85, 0.1, 0.075]), "WaPo")
            wapo_button.on_clicked(self.wapo)
            run_button = Button(plt.axes([0.6, 0.85, 0.1, 0.075]), "Run")
            run_button.on_clicked(self.run)
            stop_button = Button(plt.axes([0.8, 0.85, 0.1, 0.075]), "Stop")
            stop_button.on_clicked(self.stop)
            while not self.closed:
                self.draw()
                plt.draw()
                plt.pause(0.05)
                await asyncio.sleep(0.01)
        finally:
            self.save_data()
            await self.ble_connection.close()


robot_display = RobotDisplay()
asyncio.run(robot_display.main())
