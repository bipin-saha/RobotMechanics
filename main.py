from visualize import *
from Transformation import *
import numpy as np
from Kinematics import *
from serialRW import write_line, read_line
import serial
import time

theta1 = 0
theta2 = 0

"""
try:
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = 'COM3'
    ser.open()
except:
    print("error")
"""

write_json = {
    "theta1" : theta1,
    "theta2" : theta2}

if __name__ == "__main__":
    while True:
    
        theta1, theta2 = inverseKinematics(link_1=95, link_2=101.79, px=100, py=90)
        theta1, theta2 = int(theta1), int(theta2)
        #print(theta1, theta2)
        write_json = {
             "theta1" : theta1,
                "theta2" : theta2}
        write_json = str(write_json)

        write_line(115200, 'COM3', write_json)

        message = read_line(115200, 'COM3')
        print(message)
        
        time.sleep(2)
