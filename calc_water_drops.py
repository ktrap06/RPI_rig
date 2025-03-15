#open solenoids to fill before session
import multiprocessing as mp
import RPi.GPIO as GPIO
from time import sleep # mktime, strptime
import time
import datetime
from datetime import datetime,timedelta
from sys import exit
import random
import gc
from os import path, mkdir
import numpy as np
import csv
import picamera

import datetime
import os

solenoidL_pin = 26
solenoidR_pin = 19

GPIO.setmode(GPIO.BCM) # Use board numbering mode
GPIO.setup(solenoidL_pin, GPIO.OUT)
GPIO.setup(solenoidR_pin, GPIO.OUT)
def dispense(solenoid, drop, t):
    """
    Opens the specified solenoid for t length of time and then closes it.
    Dependent on if the capacitive sensor is touched and will only
    dispense for allocated time t. The solenoid will stay closed if the capacitive
    sensor is touched outside the allocated time.
    """
    for i in range(1,drop+1):
        start_time = time.time() # record start time
        
        touched = False # flag to check if capacitive sensor is touched within allocated time
        if solenoid == 'L':
            solenoid_pin = solenoidL_pin
        elif solenoid == 'R':
            solenoid_pin = solenoidR_pin
        
        GPIO.output(solenoid_pin, GPIO.HIGH)
        #print("Solenoid", str(solenoid_pin), "opened at", str(time.time()))
        time.sleep(t)
        GPIO.output(solenoid_pin, GPIO.LOW)
        #break # break the loop if solenoid has been opened
            
    

drops = int(input("How many drops? " ))
t = float(input("How long for each drop? " ))
sol = input("which solenoid? L or R: " )

dispense(sol,drops,t)

