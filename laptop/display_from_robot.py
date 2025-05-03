import arena
import asyncio
from datetime import datetime
import json
from pprint import pprint
import numpy as np
from math import pi
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import pickle
from geom2d import pt_coords
from robot_ble_connection import BleConnection
import struct

# Offset + sensor value = actual distance to robot center
OFFSETS = [19, 24, 15, 15, 15, 10, 20]

waypoints_file = "waypoints.txt"
data_file = "saved_data.pkl"

def read_waypoints(wp_file):
    """Load waypoints from file"""
    waypoints = []
    with open(wp_file) as f:
        lines = f.readlines()
        for line in lines:
            if ',' in line:
                str_x, str_y = line.split(',')
                wp = float(str_x), float(str_y)
                waypoints.append(wp)
    return waypoints

waypoints = read_waypoints(waypoints_file)


class RobotDisplay:
    def __init__(self):
        self.ble_connection = BleConnection(self.handle_data)
        self.buffer = ""
        self.arena = {"arena": arena.boundary_lines,}
        self.wp_list = waypoints
        self.waypoints = np.array(self.wp_list, dtype=np.float32)
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
            if "wp_len" in message:  # robot reporting len(waypoints)
                # This is the first message sent from PicoBot
                now = datetime.now()
                correct_len = len(self.wp_list)
                robot_len = message["wp_len"]
                if robot_len == correct_len:
                    print(f"All {correct_len} waypoints sent to robot", now)
                else:
                    print(f"Robot has {robot_len} points, should have {correct_len}")

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

                """
                # generate (fictional) 1500 mm perimeter points from all sensors
                perimeter_points = []
                for idx in range(7):
                    rel_angle = pi/2 - idx * pi/6
                    xy_coords = pt_coords(pose, 1.5, rel_angle)
                    perimeter_points.append(xy_coords)
                # list only 5 forward looking sensors (remove A & G)
                self.fwd_pnts_list = perimeter_points[1:-1]
                self.fwd_pnts = np.array(self.fwd_pnts_list, dtype=np.float32)
                """

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
        """
        if self.fwd_pnts is not None:
            self.axes.scatter(self.fwd_pnts[:,0], self.fwd_pnts[:,1], color="cyan")
        """
        
    async def send_waypoint(self, point):
        wp_req = "!W".encode('utf8') + (json.dumps(point) + "\n").encode()
        await self.ble_connection.send_uart_data(wp_req)

    async def send_command(self, code):
        request = (code + "\n").encode('utf8')
        print(f"Sending request: {request}")
        await self.ble_connection.send_uart_data(request)

    def auto(self, _):
        self.button_task = asyncio.create_task(self.send_command("!A"))

    def tele(self, _):
        self.button_task = asyncio.create_task(self.send_command("!T"))

    def run(self, _):
        self.button_task = asyncio.create_task(self.send_command("!R"))

    def stop(self, _):
        self.button_task = asyncio.create_task(self.send_command("!S"))

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
            print("Sending waypoints to robot")
            for point in waypoints:
                await self.send_waypoint(point)
            print("Confirming receipt of all waypoints")
            await self.send_command("!C")
            self.fig.canvas.mpl_connect("close_event", self.handle_close)
            tele_button = Button(plt.axes([0.2, 0.85, 0.1, 0.075]), "Tele")
            tele_button.on_clicked(self.tele)
            auto_button = Button(plt.axes([0.4, 0.85, 0.1, 0.075]), "Auto")
            auto_button.on_clicked(self.auto)
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
