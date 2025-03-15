"""v6"""

"""
updates:
- toggle spout moving
"""

"""
stage 3 discrimination paradigm. It presents stim 1 or stim 2 randomly,
the mouse chooses the associated correct spout and if they are right
they recieve a reward from that spout as the other spout moves away.

Sidebias correction is included where the 10 trials preceeding will affect
which stim is more likely to be presented
"""

print("stage 3 version 6")

#importing libraries
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
import time
from picamera2 import Picamera2, Preview
import datetime
import os
import board #v4 #mpr121
import busio #v4 #mpr121
import adafruit_mpr121 #v4
i2c = busio.I2C(board.SCL, board.SDA) #v4
mpr121 = adafruit_mpr121.MPR121(i2c) #v4 # Create MPR121 object
from pwm import PWM #v5
buzzer = PWM(1) #v5  #PWM1 defaults to GPIO19 which is where the buzzer should be connected
buzzer.export()  #v5

"""settings, factors that can be changed between trials                         
---------------------------------------------------------------------------------"""
num_trials = 100        #number of total trials
sleep_time = False       #include the 30 second sleep time and 10 second dark frames?
stim_length = 0.12       #length of single stim, double is divided in two
ITI = 1               #inter time interval, represents the time between the first and second stim of double stim condition
sideBias = True         #include side bias correction?
t_Presentation = 2      #total time they have to decide and lick (spouts presented)
t_Decide = 1.2          #time between stim and spout presentation
solenoidL_time = 0.2    #length of time water is released (solenoids are open for)
solenoidR_time = 0.2    #length of time water is released (solenoids are open for)
touch_threshold = 19    #MPR121 capacitive touch sensor threshold, for initial touch
release_touch_threshold = 4    #MPR121 capacitive touch sensor release threshold, to release detection
spout_move = True
"""---------------------------------------------------------------------------------"""


# Set up GPIO pins
LED_PIN = 18
solenoidL_pin = 26
solenoidR_pin = 13
spoutL_pin = 20
spoutR_pin = 21
touchR_pin = 9 #v4
touchL_pin = 7 #v4
cortical_pin = 25
num_licks = 0


#setting touch sensor threshold
for i in range(12): 
    mpr121[i].threshold = touch_threshold          #v4 #set values in settings
    mpr121[i].release_threshold = release_touch_threshold   #v4 #set values in settings

GPIO.setmode(GPIO.BCM) # Use board numbering mode
GPIO.setup(solenoidL_pin, GPIO.OUT)
GPIO.setup(solenoidR_pin, GPIO.OUT)
GPIO.setup(spoutL_pin, GPIO.OUT)
GPIO.setup(spoutR_pin, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT) # Set pin 18 as an output pin
GPIO.setup(cortical_pin, GPIO.OUT)
GPIO.output(cortical_pin, GPIO.LOW)
#pi camera set up
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(
    main={"size": (500, 500), "format": "YUV420"},
    controls={"FrameDurationLimits": (11111, 11111)}
)
picam2.configure(camera_config)

"visual stimulation function"
def visStim(n=1, duration=0.5):
    """
    visual discrimination stimulus, where n is the number of flashes,
    and duration is the length of time in seconds.
    the time flashing is equal to the intertime interval (iti)
    """
    
    
    if n == 1: #stim 1
        GPIO.output(LED_PIN, GPIO.HIGH) # Turn on the LED
        sleep(stim_length)
        GPIO.output(LED_PIN, GPIO.LOW) # Turn off the LED
        #sleep(duration)
    else: #stim 2
        GPIO.output(LED_PIN, GPIO.HIGH) # Turn on the LED
        sleep(stim_length)
        GPIO.output(LED_PIN, GPIO.LOW) # Turn off the LED
        sleep(duration)
        GPIO.output(LED_PIN, GPIO.HIGH) # Turn on the LED
        sleep(stim_length)
        GPIO.output(LED_PIN, GPIO.LOW) # Turn off the LED

"solenoid dispensing function"
def dispense(solenoid, t):
    global num_licks
    """
    Opens the specified solenoid for t length of time and then closes it.
    Dependent on if the capacitive sensor is touched and will only
    dispense for allocated time t. The solenoid will stay closed if the capacitive
    sensor is touched outside the allocated time.
    """
    start_time = time.time() # record start time
    
    touched = False # flag to check if capacitive sensor is touched within allocated time
    if solenoid == 'L':
        solenoid_pin = solenoidL_pin
        touch_pin = touchL_pin
        solenoid_time = solenoidL_time
    else:
        solenoid_pin = solenoidR_pin
        touch_pin = touchR_pin
        solenoid_time = solenoidR_time
    while(time.time()-start_time) < t:
        if mpr121[touch_pin].value: #v4
            #touched = True # set flag to true
            touched_time = time.time()
            time.sleep(solenoid_time/2)
            GPIO.output(solenoid_pin, GPIO.HIGH)
            #print("Solenoid", str(solenoid_pin), "opened at", str(time.time()))
            time.sleep(solenoid_time/2)
            GPIO.output(solenoid_pin, GPIO.LOW)
            #break # break the loop if solenoid has been opened
            num_licks +=1

def playTone(freq, n=1, duration=0.2, iti=0.1):
    """
    Plays one or more tones of a specified frequency and duration and inter-tone interval.
    freq is a str indicating either a Hi, Med or Low tone
    n is the number of repetitions (defaults to 1)
    duration is length in seconds of the tone (defaults to 0.2)
    iti is the break between tones in seconds (defaults to 0.1)
    """
    freq_dict = {'Hi': 12000, 'Med': 10000, 'Low': 8000}
    period = int(1000000000/freq_dict[freq])
    duty = int(period/2)
    if buzzer.duty_cycle > period:
        buzzer.duty_cycle = duty
        buzzer.period = period
    else:
        buzzer.period = period
        buzzer.duty_cycle = duty
    while n > 0:
        buzzer.enable = True
        sleep(duration)
        buzzer.enable = False
        n-=1
        if n > 0:
            sleep(iti)
    
"""
main
"""
def main():
    global num_licks #over the whoel trial counts the number of times the capacitive sensor was licked in the given time
    try:
        GPIO.output([spoutL_pin,spoutR_pin], GPIO.HIGH) #move spouts in to adjust them
        input('Adjust the Spouts and Press Enter...')
        if spout_move == True:
            GPIO.output([spoutL_pin,spoutR_pin], GPIO.LOW) #move spouts out
        cage_id = input('cage ID: ')
        if cage_id == "test":
            mousename = "test"
            stage = "test"
            daye = "test"
        else:
            mousename = input('indvidual mouse: ')
            stage = input('stage #: ')
            daye = input('day of stage: ') 
        
        if not os.path.exists('/home/pi/PiDisc/data/%s/%s_%s_%s/Videos/' %(cage_id, stage, cage_id, mousename)): #puts it into the folder
            print('Directory for mouse or cage does not exists. Creating directory...')
            os.makedirs('/home/pi/PiDisc/data/%s/%s_%s_%s/Videos/' %(cage_id, stage, cage_id, mousename)) #creates the folder if it is not already there
        print('Numbber of trials set at:',num_trials)
        print('Reminder: Start recording on DALSA Cortical Camera')

        now = datetime.datetime.now()
        date = now.strftime("%Y-%m-%d_%H-%M-%S") #makes sure the date is without colons for easy file transfering

        picam2.start_preview(Preview.QTGL)
        picam2.start_and_record_video('/home/pi/PiDisc/data/%s/%s_%s_%s/Videos/%s_%s_%s_%s_%s_.mp4' % (cage_id, stage, cage_id, mousename, cage_id, mousename, stage, daye, date))

        
        if sleep_time == True:
            print('10 second blackout period starting...')
            sleep(10)
        else:
            print('10 second blackout period skipped...')
        data = [] #for the timeseries data point
        trial_data = [] #for the sidebias calculation (held in memory and not written)
        t_choice = 0
        #Cortical & Infrared LED ON
        t_ctx_on = time.time()
        print('cortical & infrared light on')
        GPIO.output(cortical_pin, GPIO.HIGH)
        data.append((t_ctx_on,10))
        if sleep_time == True:
            print('30 second buffer period starting...')
            sleep(30)
        else:
            print('30 second buffer period skipped')
            
        with open('/home/pi/PiDisc/data/%s/%s_%s_%s/%s_%s_%s_%s_%s_timeseries.csv' %(cage_id, stage, cage_id, mousename, cage_id, mousename, stage, daye, date), 'a', newline='') as file:
            for pair in data:
                file.write('\t'.join(map(str,pair)) + '\n') #writes in the first data.append into timeseries file
        
        with open('/home/pi/PiDisc/data/%s/%s_%s_%s/%s_%s_%s_%s_%s_metadata_settings.txt' %(cage_id, stage, cage_id, mousename, cage_id, mousename, stage, daye, date), 'a') as file:
            meta = ('total trial length: '+ '\t' + str(num_trials) + '\n' +
                    'length of single flash: ' + '\t' + str(stim_length)+ '\n' +
                    'double flash interval: ' + '\t' + str(ITI) + '\n' +
                    'time spouts are presented: ' + '\t' + str(t_Presentation) + '\n' +
                    'time between stim and spout present: ' + '\t' + str(t_Decide) + '\n' +
                    'time L solenoids are open: ' + '\t' + str(solenoidL_time) + '\n' +
                    'time R solenoids are open: ' + '\t' + str(solenoidR_time) + '\n' +
                    'capacitive touch threshold: ' + '\t' + str(touch_threshold) + '\n' +
                    'capacitive touch release threshold: ' + '\t' + str(release_touch_threshold) + '\n'
                    )
            file.write(str(meta))
        
        sleep(1)
        
        
        print('Starting the session...')
        
        ncorrect_left = 0
        ncorrect_right = 0
        ntotal_left = 0
        ntotal_right = 0
        m = 0.8  # Choose an appropriate value for m
        recent_trials = []  # To store the outcomes of the last 20 trials

        for n in range(1,num_trials+1):
            
            data = []
            session_data = []
            lickedR = False
            lickedL = False


            print('start trial #:',n) 
            
            #stop if the capacitive sensor is stuck
            sensor1_error = False
            if mpr121[touchL_pin].value or mpr121[touchR_pin].value: #v4
                sensor1_error = True
                print('sensor error (1): pausing trial...')
            while mpr121[touchL_pin].value or mpr121[touchR_pin].value: #v4
                GPIO.output([spoutL_pin,spoutR_pin], GPIO.LOW)
                sleep(0.02)
            if sensor1_error == True:
                print('error resolved(1): continuing trial...')
            
            sleep(3)
            
            #buzzer signaing start of trial
            data.append((time.time(),0))
            playTone('Med', n=1, duration=0.1, iti=1)
            
            sleep(1.2)
            stim = random.choice([1,2]) #either stim 1 or 2
            if sideBias and len(recent_trials) > 10: #uses the 10 more recent trials to calculate
                r = random.uniform(-1, 1) #number between -1 and 1 is used and the threshold is between those numbers
                print('SideBias choice threshold: '+ str(tb))
                if r > tb:
                    stim = 1
                else:
                    stim = 2

            data.append((time.time(),stim))
            t_start = time.time()
            if stim == 1:
                visStim(1,ITI)
                ntotal_left += 1
                print('stim 1 presented')
                
            else:
                visStim(2,ITI)
                ntotal_right += 1
                print('stim 2 presented')
                
            sleep(t_Decide)
            data.append((time.time(),3))
            if spout_move == True:
                GPIO.output([spoutL_pin,spoutR_pin], GPIO.HIGH)
            t_decision = time.time()
            while not lickedL and not lickedR and time.time()-t_decision < t_Presentation: 
                if mpr121[touchL_pin].value: #v4
                    print('Left Lick')
                    data.append((time.time(),4))
                    t_choice = time.time()
                    if spout_move == True:
                        GPIO.output(spoutR_pin, GPIO.LOW) # Right spout goes back
                    lickedL = True
                elif mpr121[touchR_pin].value: #v4
                    print('Right Lick')
                    data.append((time.time(),5))
                    t_choice = time.time()
                    if spout_move == True:
                        GPIO.output(spoutL_pin, GPIO.LOW) # Left spout goes back
                    lickedR = True
                    
            if [stim,lickedL,lickedR] in [[1,1,0],[2,0,1]] : #if they lick the correctly associated spout them they get a reward
                success = True
                fail = False
                r=1
                data.append((time.time(),6))
                if lickedL:
                    dispense('L',(t_Presentation - (t_choice - t_decision)))
                    ncorrect_left += 1   
                    print('left success')

                elif lickedR:
                    dispense('R',(t_Presentation - (t_choice - t_decision)))
                    ncorrect_right += 1
                    print('right success')

            elif [stim,lickedL,lickedR] in [[2,0,0],[1,0,0]]:
                success = False
                fail = True
                t_choice = 0
                r=0
                print('fail')
            else:
                success = False
                fail = True
                r= 0
                print('fail')

            tb = min(m, max(-m, (ncorrect_left / max(1, ntotal_left)) - (ncorrect_right / max(1, ntotal_right))))
            recent_trials.append([stim, success])
            if len(recent_trials) > 20:
                old_stim, old_success = recent_trials.pop(0)
                if old_success:
                    if old_stim == 1:
                        ncorrect_left -= 1
                    elif old_stim == 2:
                        ncorrect_right -= 1
                if old_stim == 1:
                    ntotal_left -= 1
                elif old_stim == 2:
                    ntotal_right -= 1

            data.append((time.time(),7))
            if spout_move == True:
                GPIO.output([spoutL_pin,spoutR_pin], GPIO.LOW)
            
            if fail == True:
                print('2 second timeout')
                sleep(2)
            else:
                print('no timeout')
                
            print('end of trial #:',n)
            
            sleep(3)
            session_data.extend([n,t_start,stim,t_choice - t_decision,r])
            
            with open('/home/pi/PiDisc/data/%s/%s_%s_%s/%s_%s_%s_%s_%s_data.txt' %(cage_id, stage, cage_id, mousename, cage_id, mousename, stage, daye, date), 'a') as file:
                file.write('\t'.join(map(str,session_data)) + '\n')
            print('--------------') 
            
            with open('/home/pi/PiDisc/data/%s/%s_%s_%s/%s_%s_%s_%s_%s_timeseries.csv' %(cage_id, stage, cage_id, mousename, cage_id, mousename, stage, daye, date), 'a') as file:
                for pair in data:
                    file.write('\t'.join(map(str,pair)) + '\n')
        
        data = []
        t_end = time.time()
        if sleep_time == True:
            print('30 second buffer period starting...')
            sleep(30)

        else:
            print('30 second buffer period skipped')
        GPIO.output(cortical_pin,GPIO.LOW) #turns off cortical light
        print('cortical light off')
        data.append((t_end,11))
        print('end of session:',mousename)
        print('Saving Video and Finalizing Data.... Do NOT exit!')
        sleep(10)
        
        with open('/home/pi/PiDisc/data/%s/%s_%s_%s/%s_%s_%s_%s_%s_data.txt' %(cage_id, stage, cage_id, mousename, cage_id, mousename, stage, daye, date), 'a') as file:
            lastline = 'CTX LED ON: '+ '\t' + str(t_ctx_on) + '\n' + 'CTX LED OFF: ' + '\t' + str(t_end)+ '\n' + 'Number of Drops: ' + '\t' + str(num_licks)+ '\n' + 'double flash interval: ' + '\t' + str(ITI)
            session_data.extend(lastline)
            file.write(str(lastline))

        with open('/home/pi/PiDisc/data/%s/%s_%s_%s/%s_%s_%s_%s_%s_timeseries.csv' %(cage_id, stage, cage_id, mousename, cage_id, mousename, stage, daye, date), 'a') as file:
            for pair in data: #in pair means it keeps the time point and the tag number paired together
                file.write('\t'.join(map(str,pair)) + '\n') 
                
        print('Saving Video...')

        picam2.stop_recording()
        picam2.stop_preview()

        print('Video saved')
        print('Data Saved Successfully!')
        
    except KeyboardInterrupt:
        print('Closing program...')
    except ValueError as err:
        print("Value Error: {0}".format(err))
    
    finally:
        print('Number of Drops: ',num_licks) #prints out the number of licks to calculate the amount of water they get
        GPIO.output(solenoidL_pin, GPIO.LOW)
        GPIO.output(solenoidR_pin, GPIO.LOW)
        GPIO.output(spoutL_pin, GPIO.LOW)
        GPIO.output(spoutR_pin, GPIO.LOW)
        GPIO.output(LED_PIN, GPIO.LOW) # Set pin 18 as an output pin
        GPIO.output(cortical_pin, GPIO.LOW)
        #GPIO.cleanup()
        picam2.stop_recording()
        picam2.stop_preview()
        # Clean up
        buzzer.enable = False
        buzzer.unexport()
        
     
if __name__ == "__main__":
    main()

