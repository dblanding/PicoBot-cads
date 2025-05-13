# drive_instrux.py

# Use a coarse value for pi
# json will struggle with too many decimal places
pi = 3.1416

wapo_list = [
    (1, -0.3),
    ]

wapo_list2 = [
    (0, 0),
    ]

instrux_list = [
    {"!TGH": pi,},
    {"!SWP": wapo_list,},
    {"!DWR": None,},
    {"!TGH": 0,},
    {"!SWP": wapo_list2,},
    {"!DWR": None,},
    {"!TGH": 0,},
    ]
