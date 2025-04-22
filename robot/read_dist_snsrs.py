# read_dist_snsrs.py
"""
Read all 7 distance sensors on a 180 degree arc
clock-wise from 9:00 to 3:00
"""

from machine import I2C, Pin, UART
from time import sleep
from dist_snsr_array import DistSnsrArray

# set up multiplexer on i2c0
i2c0 = I2C(0, sda=Pin(12), scl=Pin(13))
print("Device Addresses found on I2C0: ",i2c0.scan())

# Instantiate distance sensor array
dsa = DistSnsrArray(i2c0)

def read_snsrs():
    return dsa.read_all()

if __name__ == "__main__":
    NMBR_OF_READS = 40
    # sensor bins
    a = []
    b = []
    c = []
    d = []
    e = []
    f = []
    g = []
    all_bins = [a, b, c, d, e, f, g]
    for _ in range(NMBR_OF_READS):
        reading = read_snsrs()
        print(reading)
        # sort readings into bins
        for i in range(len(reading)):
            all_bins[i].append(reading[i])
        sleep(0.25)

    for bin in all_bins:
        print(f"Max = {max(bin)}")
        print(f"Min = {min(bin)}")
        print(f"Avg = {sum(bin)/len(bin)}")
        print("--------------------")