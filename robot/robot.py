# robot.py (aka main.py)
"""
MicroPython code for PicoBot project
* Raspberry Pi Pico mounted on differential drive robot
* 56:1 gear motors
* 1 BLE UART Friend module for bi-directional comm w laptop
* Circular Array of Distance Sensors (VL53L0X)
    * distance sensor units: mm (gets converted to meters in mapper)
* SparkFun OTOS reports pose (x, y, heading)
    * initially (0, 0, 0)
    * x & y units: meters
    * heading units: radians, Zero along X-axis, pos CCW, neg CW
    * yaw_rate (rad/sec) also used
"""

import asyncio
import gc
import json
from machine import I2C, Pin, UART
from math import pi, sqrt, atan2, sin, cos
import motors
from parameters import JS_GAIN, MIN_DIST, ANGLE_TOL, MIN_ANGLE
from parameters import P_TURN_GAIN, D_TURN_GAIN, MAX_ANG_SPD
import qwiic_i2c
import qwiic_otos
import struct
import time
import VL53L0X
from dist_snsr_array import DistSnsrArray

D_GAIN = 0.5  # Gain of Derivative feedback term

# set up uart for communication with BLE UART friend
print("Setting up uart for sending robot data to laptop")
uart = UART(0, 9600)
uart.init(tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1, timeout=10)

# setup onboard LED
led = machine.Pin("LED", machine.Pin.OUT)

# Initialize I2C1 using qwiic library
i2c1 = qwiic_i2c.get_i2c_driver(sda=14, scl=15, freq=100000)

# set up OTOS on i2c1
myOtos = qwiic_otos.QwiicOTOS(23, i2c1)

# set up multiplexer on i2c0
i2c0 = I2C(0, sda=Pin(12), scl=Pin(13))
print("Device Addresses found on I2C0: ",i2c0.scan())

# Instantiate distance sensor array
dsa = DistSnsrArray(i2c0)

# Initialize I2C1 using qwiic library
my_bus = qwiic_i2c.get_i2c_driver(sda=14, scl=15, freq=100000)

# Set up and initialize Optical Tracking Odomety Sensor (OTOS) device
myOtos = qwiic_otos.QwiicOTOS(23, my_bus)
print("\nSetting up OTOS\n")

# Perform the self test
result = myOtos.selfTest()
    
# Check if the self test passed
if(result == True):
    print("Self test passed!")
else:
    print("Self test failed!")

# Check if it's connected
if not myOtos.is_connected():
    print("The OTOS isn't connected", file=sys.stderr)

myOtos.begin()

print("Ensure the OTOS is flat and stationary during calibration!")
for i in range(5, 0, -1):
    print("Calibrating in %d seconds..." % i)
    time.sleep(1)

print("Calibrating IMU...")

# Calibrate the IMU, which removes the accelerometer and gyroscope offsets
myOtos.calibrateImu()

# Account for OTOS location w/r/t robot center
offset = qwiic_otos.Pose2D(1.25, 0, 0)
myOtos.setOffset(offset)

# Set units for linear and angular measurements.
# If not set, the default is inches and degrees.
# Note that this setting is not stored in the sensor.
# it's part of the library, so you need to set it.
myOtos.setLinearUnit(myOtos.kLinearUnitMeters)
myOtos.setAngularUnit(myOtos.kAngularUnitRadians)

# Reset the tracking algorithm - this resets the position to the origin,
# but can also be used to recover from some rare tracking errors
myOtos.resetTracking()

print("OTOS initialized")

def get_pose():
    pose = myOtos.getPosition()
    return (pose.x, pose.y, pose.h)

def bot_moving(pose0, pose1):
    """Detect if bot has moved more than MIN_DIST from pose0 to pose1
    or if its angle has changed by more than MIN_ANGLE"""
    x0, y0, h0 = pose0
    x1, y1, h1 = pose1
    d = sqrt((x0 - x1)**2 + (y0 - y1)**2)
    a = abs(h1 - h0)
    return d > MIN_DIST or a > MIN_ANGLE

def check_pose(prev_pose):
    curr_pose = get_pose()
    is_moving = bot_moving(prev_pose, curr_pose)
    return (is_moving, curr_pose)

def get_yaw_rate():
    """https://github.com/sparkfun/qwiic_otos_py/tree/master/examples"""
    vel = myOtos.getVelocity()  # gyro data
    hdg_rate = vel.h  # rad/sec
    return hdg_rate

def send_json(data):
    uart.write((json.dumps(data) + "\n").encode())

def read_json():
    try:
        data = uart.readline()
        decoded = data.decode()
        return json.loads(decoded)
    except (UnicodeError, ValueError):
        print("Invalid data")
        return None

# geometry helper functions
def p2r(r, theta):
    """Convert polar coords to rectangular"""
    x = cos(theta) * r
    y = sin(theta) * r
    return (x, y)

def r2p(x, y):
    """Convert rectangular coords to polar"""
    r = sqrt(x*x + y*y)
    theta = atan2(y, x)
    return (r, theta)

def rel_polar_coords_to_pt(curr_pose, point):
    """Based on current pose, return relative
    polar coords dist (m), angle (rad) to goal point.
    """
    # current pose
    x0, y0, a0 = curr_pose

    # coords of goal point
    x1, y1 = point

    # Relative coords to goal point
    x = x1 - x0
    y = y1 - y0

    # Convert rectangular coords to polar
    r, theta = r2p(x, y)

    # Relative angle to goal point
    rel_angle = theta - a0

    # ensure angle is between -pi/2 and +pi/2
    if rel_angle < -pi:
        rel_angle += 2 * pi
    elif rel_angle > pi:
        rel_angle -= 2 * pi

    return (r, rel_angle)


class Robot():
    def __init__(self):
        self.lin_spd = 0.7  # nominal drive speed
        self.ang_spd = 0  # prev value ang_spd only when stuck
        self.run = True
        self.mode = 'IDL'  # Idle
        self.errors = []
        self.prev_pose = (0, 0, 0)
        self.prev_time = None
        self.waypoint = None
        self.goal_heading = None
        self.cum_angle = None
        self.goal_angle = None

    def turn_to_heading(self, goal_heading, gz, yaw):
        """
        Return ang_spd needed to drive motors when
        turning in place to goal_angle (radians).
        self.ang_spd is used to remember prev ang_spd when stuck
        """
        # calculate ang_spd to steer to goal_heading
        yaw_err = yaw - goal_heading
        p = -(yaw_err * P_TURN_GAIN)  # proportional term
        d = -(gz * D_TURN_GAIN)  # derivative term
        ang_spd = p + d

        # limit value of ang_spd
        if ang_spd < -MAX_ANG_SPD:
            ang_spd = -MAX_ANG_SPD
        if ang_spd > MAX_ANG_SPD:
            ang_spd = MAX_ANG_SPD

        # reset self.ang_spd to zero if not stuck
        if abs(gz) > 0.01:
            self.ang_spd = 0

        # check if turn is complete
        if abs(gz) < 0.01 and abs(yaw_err) < ANGLE_TOL:
            ang_spd = 0

        # give an extra boost if needed to overcome static friction
        elif abs(gz) < 0.01 and abs(yaw_err) < 3 * ANGLE_TOL:
            if not self.ang_spd:  # not previously boosted
                ang_spd *= 1.5
                self.ang_spd = ang_spd
            else:  # boost it further
                ang_spd = self.ang_spd * 1.5
                self.ang_spd = 0

        return ang_spd

    def stop(self):  # stop moving
        motors.drive_motors(0, 0)

    def end(self):  # shut down program
        self.run = False

    async def main(self):
        while self.run:
            try:
                # read distances from VCSEL sensors
                distances = dsa.read_all()

                # check current pose
                is_moving, pose = check_pose(self.prev_pose)
                self.prev_pose = pose
                
                # get yaw (rad) from pose and yaw rate (rad/sec) from gyro
                _, _, yaw = pose
                gz = get_yaw_rate()

                curr_time = time.ticks_ms()
                if not self.prev_time:  # initialize
                    self.prev_time = curr_time
                    continue
                elif self.mode == 'TRA':  # Turn in place by a relative angle (+ is CCW)
                    delta_time = (curr_time - self.prev_time)/1000  # (sec)
                    self.prev_time = curr_time
                    self.cum_angle += (gz * (delta_time))
                    if self.goal_angle > 0:
                        if self.cum_angle < self.goal_angle:
                            motors.drive_motors(0, MAX_ANG_SPD)
                        else:  # Completed CCW turn to rel goal angle
                            motors.move_stop()
                            send_json({"status": "READY"})
                            self.mode = 'IDL'
                    elif self.goal_angle <= 0:
                        if self.cum_angle > self.goal_angle:
                            motors.drive_motors(0, -MAX_ANG_SPD)
                        else:  # Completed CW turn to rel goal angle
                            motors.move_stop()
                            send_json({"status": "READY"})
                            self.mode = 'IDL'

                elif self.mode == 'TGH':  # Turn in place to a global heading
                    ang_spd = self.turn(self.goal_heading, gz, yaw)
                    motors.drive_motors(0, ang_spd)
                    if not ang_spd:  # arrived at goal heading
                        motors.move_stop()
                        send_json({"status": "READY"})
                        self.mode = 'IDL'

                elif self.mode == 'DWP':  # Drive to waypoint
                    goal_dist, goal_angle = rel_polar_coords_to_pt(pose, self.waypoint)
                    if goal_dist > 0.2:
                        # drive to waypoint, steering to goal_angle
                        d = -(gz * D_GAIN)  # derivative term
                        ang_spd = goal_angle + d
                        motors.drive_motors(self.lin_spd, ang_spd)
                    else:
                        # arrived at waypoint
                        motors.move_stop()
                        send_json({"status": "READY"})
                        self.mode = 'IDL'

                if is_moving:  # If robot is moving, send robot data to laptop
                    if self.mode == 'DWP':
                        send_json({
                            "pose": list(pose),
                            "distances": distances,
                            "errors": self.errors,
                            "goal_dist": goal_dist,
                            "goal_angle": goal_angle,
                            })
                    else:
                        send_json({
                            "pose": list(pose),
                            "distances": distances,
                            "errors": self.errors,
                            })

                led.toggle()
                await asyncio.sleep(0.1)

            except Exception as e:
                self.errors.append(e)

            finally:
                motors.move_stop()


async def command_handler(robot):
    print("Starting handler")
    robot_task = None
    # robot_task = asyncio.create_task(robot.main())
    while True:
        if uart.any():
            try:
                # Handle Bluetooth request
                bytestring = uart.readline()
                cmd = bytestring[:4].decode('utf8')
                if cmd == '!RUN':
                    print("Received Run request, starting robot")
                    if not robot_task:
                        robot_task = asyncio.create_task(robot.main())
                elif cmd == '!DWP':
                    point = json.loads(bytestring[4:])
                    robot.waypoint = point
                    print("Received DWP request")
                    robot.mode = 'DWP'
                elif cmd == '!TGH':
                    robot.goal_heading = json.loads(bytestring[4:])
                    print("Received TGH request")
                    robot.mode = 'TGH'
                elif cmd == '!TRA':
                    robot.goal_angle = json.loads(bytestring[4:])
                    robot.cum_angle = 0
                    print("Received TRA request")
                    robot.mode = 'TRA'
                elif cmd == '!STP':
                    print("Received STP request")
                    robot.stop()
                elif cmd == '!END':
                    print("Received END request")
                    robot.end()
                

            except Exception as e:
                robot.errors.append(e)
        
        await asyncio.sleep(0.1)


robot = Robot()
asyncio.run(command_handler(robot))
