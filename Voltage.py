"""Sample code and test for adafruit_in219"""

import logging
from ina219 import INA219
import time

SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 0.2

ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, log_level=logging.INFO, address=0x42, busnum=1)
ina.configure(ina.RANGE_16V, ina.GAIN_AUTO)

def measure():
    return (ina.voltage())

if __name__ == '__main__':
    while True:
        time.sleep(1)
        print(ina.voltage())

