#  Uses python interface for librealsense v1.0 to integrate recording of
#  depth, colour, and humidity data at 10Hz or less. For high speed recordings,
#  refer to TermiteScan. Requires libftdi installation, Adafruit_GPIO python
#  library, and probably a whole bunch of other things.
#
#  Since true threading in python requires some hacking, this program does 
#  not attempt to thread image saving subfunctions. This means its timestamping is
#  only accurate to 25ms, due to overhead.
#  Recommend using C implementation for recordings > 10Hz
# 
# 
#  to do:
#        get keyboard input working 
#        write a librealsense2 version for maximal utility, attempt to 
#        get a high-speed version working for 20Hz+ 

import logging
logging.basicConfig(level=logging.INFO)

import time
import datetime as dtime
import numpy as np
import cv2
import io
import os
import threading

import Adafruit_GPIO.FT232H as ft
import Adafruit_GPIO as GPIO
import FT_TCA9548A as ft_TCA
import SHTConstants as sc

import pyrealsense as pyrs

def control_loop(hum_av, hum_setpoint, board, pin):
    global humFlag
    # compare humidity average with desired setpoint
    logging.info("Checking humidity ...")
    hum_error = hum_setpoint - hum_av
    if hum_error < 0 and humFlag: # overshot, turn off
        logging.info("Reached setpoint, turning off humidifier")
        turn_off_humidifier(board, pin)
        humFlag = False
    elif hum_error > 2 and not humFlag: # still gotta go up
        logging.info("Turning on humidifier")
        turn_on_humidifier(board, pin)
        humFlag = True

    #return humFlag


def turn_off_humidifier(board, pin):
    hum_control_pulse(board, pin, 0.1)


def turn_on_humidifier(board, pin):
    hum_control_pulse(board, pin, 0.1)
    time.sleep(5)
    hum_control_pulse(board, pin, 0.1)


def hum_control_pulse(board, pin, duration):
    logging.info("relay pulse")
    board.output(pin, GPIO.HIGH) # pulse high
    time.sleep(duration)
    board.output(pin, GPIO.LOW) # return to low


# initialise frame numbers
cnum = 1000000;
dnum = 1000000;

print("Warning: double check relay status and operation before using for experimentation")
time.sleep(3)

runNum = raw_input("Please enter a unique numerical ID for this recording: ")
# create filepaths
today = dtime.date.today()
colpath = today.isoformat() + "/RGB_" + runNum + "/"
depthpath = today.isoformat() + "/D_" + runNum + "/"

# check if path exists
try:
    os.makedirs(colpath)
    os.makedirs(depthpath)
except:
    print("Warning: you have chosen an existing recording ID, stored data may be overwritten")

framerate = raw_input("Please enter a framerate in frames per second [default = 1fps]: ")

if not framerate:
    colfps = 1
    depthfps = 1
else:
    colfps = int(framerate)
    depthfps = int(framerate)

humidity_rate = raw_input("Please enter sensing interval between humidity readings [default = 2 minutes] : ")
if not humidity_rate:
    humidity_period = 120
else:
    humidity_period = 60*float(humidity_rate)

hum_sp = raw_input("Enter humidity setpoint: ")
if not hum_sp:
    print("Warning, default setpoint chosen (85%RH)")
    humid_setpoint = 85
else:
    humid_setpoint = int(hum_sp)

logging.info("Humidity period set to " + str(humidity_period) + " seconds")
humchan_list = [2,4,]
print("Attempt to access sensors on multiplexor channels " + str(humchan_list) + " - change code to update" )
hum_filename = today.isoformat() + "/HumiditySensorOutput_" + runNum + ".txt"

recflag = 0

# Initialise I2C bus
ft.use_FT232H()
ft_I2C = ft.FT232H()

# Set up TCA device instance
tca = ft_TCA.TCA9548A(ft_I2C, 0x70)

# Set up humidity controller pins
control_pin = 8

ft_I2C.setup(control_pin, GPIO.OUT)

# Check relay operation, NO/NC status and pulse order 
ft_I2C.output(control_pin, GPIO.LOW)
humFlag = False

with pyrs.Service() as serv:
    with serv.Device() as dev:
        # print('Using device ' + dev)

        # set up humidity sensors
        sht_list = []
        for chanNum in humchan_list:
            sht_list.append(tca.tca_init(chanNum))
            time.sleep(0.01)

        for sensor in sht_list:
            chan = humchan_list(sht_list.index(sensor))
            tca.tca_select(chan)
            check_status = sensor.readStatus()
            logging.info("SHT Status return : " + str(check_status))
            time.sleep(0.01)
            sensor.softReset()
            sensor.clearFlags()

        # set up frame spacing
        c_interval = np.floor(1000/colfps)
        hum_incr = time.time()

        c_incr = time.time()

        while True:
            dev.wait_for_frames()
            cstamp = time.time()

            cframe = dev.color
            dframe = dev.depth
            cframe= cv2.cvtColor(cframe, cv2.COLOR_RGB2BGR)
            d_display = dframe*dev.depth_scale*1000
            d_display = cv2.applyColorMap(d_display.astype(np.uint8), cv2.COLORMAP_RAINBOW)

            cd = np.concatenate((cframe, d_display), axis=1)
            cv2.imshow('', cd)

            if ((cstamp - c_incr)*1000.0  >= c_interval) and recflag:

                print(cstamp-c_incr)
                cv2.imwrite(colpath + "ColSnap_" + str(cnum) + ".jpg", cframe)

                doutput = bytearray(dframe)
                dname = depthpath + "DepthSnap_" + str(dnum) + ".dat"
                dfile = io.open(dname, 'w+b')
                dfile.write(doutput)
                dfile.close()

                cnum += 1
                dnum += 1
                c_incr = time.time()
 

            if (cstamp-hum_incr) >= humidity_period:
                logging.info("reached humidity sensing")
                # open file, print out, close file
                with open(hum_filename, 'a+') as humfile:
                    sensNum = 0
                    humid_av = 0
                    for sensor in sht_list:
                        try:
                            chan = humchan_list(sht_list.index(sensor))
                            tca.tca_select(chan)
                            temp, humid = sensor.SingleMeasurement(sc.SINGLE_MEAS_HIGH, 2)
                            if humid < 5:
                                print("Error in sensor " +str(sensNum+1) + " , disregarding...")
                                continue

                            sensNum += 1
                            humfile.write("Frame, " + str(cnum) + ", Sensor, " + str(sensNum) + ", Temperature , " + str(temp) + " , Humidity , " + str(humid))
                            humid_av += humid

                        except: 
                            print("Check humidity sensors: error with sensor " + str(sensNum))

                hum_incr = time.time()
                # thread this
                if sensNum:
                    humid_av /= sensNum
                    threadFunc = threading.Thread(target = control_loop, args = (humid_av, humid_setpoint, ft_I2C, control_pin, ))
                    threadFunc.start()
#control_loop(humid_av, humid_setpoint, ft_I2C, control_pin, humFlag)
                    print("Humidity reading: " + str(humid_av))
                    print("Humidifier status: " + str(humFlag))
            
                else:
                    print("Problems with humidity sensors, turning off humidifier ...")
                    if humFlag:
                        turn_off_humidifier(ft_I2C, control_pin)
                        humFlag = False

            keycheck = cv2.waitKey(1) 

            if keycheck & 0xFF == ord('m'):
                recflag = 1
                print("Now recording ...")
  
            elif keycheck & 0xFF == ord('e'):
                recflag = 0
                print("Stopping recording")

            elif keycheck & 0xFF == ord('q'):
                break

