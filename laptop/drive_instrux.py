# drive_instrux.py

# Use a coarse value for pi
# json will struggle with too many decimal places
pi = 3.1416

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
    {"!TGH": pi,},
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
    {"!TGH": 0.0,},
    ]
