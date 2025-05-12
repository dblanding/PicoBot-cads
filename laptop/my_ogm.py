import matplotlib.pyplot as plt
import numpy as np
import pickle

# OGM dimensions (m) w/r/t home (0, 0) position
RES = 0.1

TOP = 4.5
BOT = -7
LFT = -7
RGT = 4.5
XHOME = int(-LFT / RES)
YHOME = int(-BOT / RES)
NROWS = int((TOP - BOT) / RES)
NCOLS = int((RGT - LFT) / RES)

def convert_point_to_ogm(pnt):
    """Convert global coords of point to row, col OGM indexes
    origin of OGM is in lower-left corner
    global origin is relatively Y_OFFSET up & X_OFFSET to right
    Each "bin" of OGM is a square of size = RESOLUTION."""
    x, y = pnt
    col_indx = int((x - LFT + RES/2) / RES)
    row_indx = int((y - BOT + RES/2) / RES)
    return col_indx, row_indx

ogm = np.zeros((NROWS, NCOLS), dtype=int)
# Make a dot at global origin
pnt = 0, 0
col_idx, row_idx = convert_point_to_ogm(pnt)
ogm[row_idx, col_idx] = 5

# Read data file and put points into ogm 'buckets'
with open('saved_data.pkl', 'rb') as file:
    data = pickle.load(file)
for key in data.keys():
    if 'pnts' in key:
        pnts = data.get(key)
        for pnt in pnts:
            col_idx, row_idx = convert_point_to_ogm(pnt)
            ogm[row_idx, col_idx] += 1

# Display ogm using matplotlib
# The extent parameter defines the coordinates that
# correspond to the corners of the displayed array.
lft = int((LFT - RES) / RES)
rgt = int((RGT - RES) / RES)
bot = int((BOT - RES) / RES)
top = int((TOP - RES) / RES)
extent = [lft, rgt, bot, top]
plt.imshow(ogm, extent=extent, origin='lower', cmap='gray_r', vmin=0, vmax=5)

# Draw 1 meter grid pattern
ts = int(1 / RES)  # Tick spacing for 1 meter intervals
lines = [line for line in range(-7 * ts, 5 * ts, ts)]
for val in lines:
    plt.axvline(val, color='gray', linewidth=0.5)
    plt.axhline(val, color='gray', linewidth=0.5)

plt.xlabel("x")
plt.ylabel("y")
plt.title(f"OGM (grid=1m, resolution={RES}m)")
#plt.colorbar()
plt.show()
