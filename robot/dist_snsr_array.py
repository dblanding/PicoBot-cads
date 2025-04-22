# dist_snsr_array.py

import struct
import time
import VL53L0X


# Multiplexer channels
# see: https://github.com/mcauser/micropython-tca9548a
CH0 = b'\x01'
CH1 = b'\x02'
CH2 = b'\x04'
CH3 = b'\x08'  # unused
CH4 = b'\x10'
CH5 = b'\x20'
CH6 = b'\x40'
CH7 = b'\x80'

# Sensor channels
SNSR_A = CH0
SNSR_B = CH1
SNSR_C = CH7
SNSR_D = CH6
SNSR_E = CH5
SNSR_F = CH4
SNSR_G = CH2
SENSORS = [SNSR_A, SNSR_B, SNSR_C, SNSR_D, SNSR_E, SNSR_F, SNSR_G]

ADDR = 0x70  # I2C address of I2C MUX

class DistSnsrArray():
    def __init__(self, bus):
        self.bus = bus
        self.tofs = self._set_up_tofs()
        self.pairs = self.set_up_pairs()

    def _set_up_tof(self, sensor):
        self.bus.writeto(ADDR, sensor)
        tof = VL53L0X.VL53L0X(self.bus)
        tof.start()
        return tof

    def _set_up_tofs(self):
        tofs = []
        for sensor in SENSORS:
            tof = self._set_up_tof(sensor)
            tofs.append(tof)
        return tofs

    def set_up_pairs(self):
        pairs = zip(self.tofs, SENSORS)
        return list(pairs)

    def _get_dist(self, tof, sensor):
        """return dist (mm) from sensor on mux"""
        self.bus.writeto(ADDR, sensor)
        dist = tof.read()
        # tof.stop()
        self.bus.writeto(ADDR, b'\x00')  # clear channels
        return dist

    def read_all(self):
        distances = []
        for pair in self.pairs:
            distance = self._get_dist(*pair)
            distances.append(distance)
        return distances
