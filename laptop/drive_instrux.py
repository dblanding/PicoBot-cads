# drive_instrux.py

# Use a coarse value for pi
# json will trip with too many decimal places
pi = 3.1416
waypoints_file = "waypoints.txt"

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

wapo_list = read_waypoints(waypoints_file)  # list

instrux_list = [
    {"!CWP": wapo_list,},
    {"!TGH": pi/2,},
    {"!TGH": -pi/2,},
    {"!TGH": pi,},
    {"!TGH": 0,},
    ]
    
instrux_list = [
    {"!DWP": wapo_list[0],},
    {"!TRA": -pi,},
    {"!DWP": wapo_list[-1],},
    {"!TGH": 0,},
    ]
