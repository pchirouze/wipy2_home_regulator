# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal
''' Boot WIPY2 '''
import os
from machine import UART

uart = UART(0, 115200)
os.dupterm(uart)

#execfile('solar_controller.py')
#execfile('regul_chauffe.py')

