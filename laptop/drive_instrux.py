# drive_instrux.py

# Sending values to the BLE central device has proven to be
# a problem if the values have too many decimal places.
# For this reason (and because it's more human-friendly)
# TGH and TRA values are expressed in int(degree) values.
# This is the only place angles are not expressed in radians.
#
# Similarly, Tele-Op joystick values are sent as integers
# then converted to (lin_spd, ang_spd) by the robot.
#
# Also, point coords are rounded to 2 decimal places max.


wapo_list0 = [
    (1.0, -0.5),
    (1.9, -1.5),
    (2.0, -2.7),
    (3.1, -3.8),
    (3.2, -5.3),
    (2.4, -6.0),
    (1.5, -6.0),
    (0.9, -5.4),
    (0.8, -4.6),
    (0.5, -4.0),
    (-1.8, -3.8),
    (-4, -3),
    ]

wapo_list = [
    (1.0, -0.5),
    (1.9, -1.5),
    (2.0, -2.8),
    (1.0, -3.6),
    (-1.8, -3.8),
    (-4, -3),
    ]


wapo_list1 = [
    (1, -0.3),
    ]

wapo_list2 = [
    (0, 0),
    ]

instrux_list = [
    {"!TGH": 180,},
    {"!SWP": wapo_list1,},
    {"!DWR": None,},
    {"!TGH": 0,},
    {"!SWP": wapo_list2,},
    {"!DWR": None,},
    {"!TGH": 0,},
    ]

instrux_list = [
    {"!SWP": wapo_list,},
    {"!DWF": None,},
    {"!TGH": 0,},
    ]
