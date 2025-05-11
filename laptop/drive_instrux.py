# drive_instrux.py

# Use a coarse value for pi
# json will struggle with too many decimal places
pi = 3.1416

wapo_list = [
    (1.85, -0.5),
    (2.35, 0),
    (2.35, 1.1)
    ]

wapo_list2 = [
    (2.35, 0),
    (1.85, -0.5),
    (0, 0),
    ]
   
instrux_list = [
    {"!SWP": wapo_list,},
    {"!DWP": None,},
    {"!TGH": -pi/2,},
    {"!SWP": wapo_list2,},
    {"!DWP": None,},
    {"!TGH": 0,},
    ]
