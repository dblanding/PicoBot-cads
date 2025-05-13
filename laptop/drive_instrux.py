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
    (1, 0.1),
    ]
   
instrux_list = [
    {"!TRA": pi,},
    {"!SWP": wapo_list2,},
    {"!DWR": None,},
    {"!TGH": 0,},
    ]
