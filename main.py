# imports
import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime as dt
import threading
from time import time
import math

# for fish finder interaction
import pynmea2
import serial

# Mode
ExternalGPS = True
FishFinder = False

# at the top so we can use it
printList = []


def fprint(data):
    global printList
    print(data)  # for terminal users...
    printList.append(data)
    return


fprint("DroneBoatMain python script started!")
if ExternalGPS:
    fprint("We are running with an external GPS plugged in!")
if FishFinder:
    fprint("We are running with a fish finder plugged in!")

# nav config
correctionforce = 0.01
distance = 0.0001
isNavigating = False
calib = False
cdist = 0
depth = 0

targetPoints = []
coordlist = []

gpsPos = (0, 0)  # this is dumb and is in X,Y (W,N) notation
imuAngle = 0
gyrototal = 0  # an unformatted version


# math functions


# function that converts radians (0 - 2pi) into degrees (0 - 360)
def degrees(radians):
    return (radians / math.pi) * 180


# converts to negative angle format
def angle(Bigangle):
    Bigangle = Bigangle % 360
    if Bigangle > 180:
        Bigangle -= 360
    return Bigangle


# converts to positive angle only format
def bigangle(Angle):
    Angle = Angle % 360
    Angle += 360
    Angle = Angle % 360
    return Angle


# function that returns the angle off of north for the given points
def getAngle(x1, y1, x2, y2):
    return degrees(math.atan2((x2 - x1), (y2 - y1)))


# gets the distance between two points
def getDistance(x1, y1, x2, y2):
    return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))


# functions for handling lists
def addPoint(x, y):
    global targetPoints
    fprint(("ADDING A POINT! " + str(x) + " " + str(y)))
    targetPoints.append((x, y))
    return


def deletePoint():
    global targetPoints
    if len(targetPoints) > 0:
        targetPoints.pop(0)
    else:
        fprint("EMPTY LIST, you probably did an overide?")
    return


# movement functions


# rotates the boat to a certain angle
def pointAtAngle(target, accuracy):
    global imuAngle
    global gpsPos
    global lasttime
    global isNavigating
    print(("Target " + str(target)))
    lasttime = dt.now().minute
    if bigangle(bigangle(target) - bigangle(imuAngle)) <= 180:
        fprint("RIGHT TURN")
        right()
    else:
        fprint("LEFT TURN")
        left()

    while abs(bigangle(target) - bigangle(imuAngle)) > accuracy and isNavigating:
        print("TURNING!")
    stop()
    fprint("DONE!")
    return


# main drive functions
def NavigateToPoint():
    global gpsPos
    global imuAngle
    global targetPoints
    global coordlist
    global cdist
    global isNavigating
    global correctionforce
    targetx = targetPoints[0][0]
    targety = targetPoints[0][1]
    #error = 0
    while getDistance(targetx, targety, gpsPos[0], gpsPos[1]) > distance and isNavigating:
        coordlist = []
        fprint("pointing at angle")
        while gpsPos[0] == 0:
            pass
        currentx = gpsPos[0]
        currenty = gpsPos[1]
        targetAngle = getAngle(currentx, currenty, targetx, targety)
        pointAtAngle(targetAngle, 5)

        forward()
        error = 0
        while getDistance(targetx, targety, gpsPos[0], gpsPos[1]) > distance and isNavigating and error < 45:
            cdist = getDistance(targetx, targety, gpsPos[0], gpsPos[1])
            fprint("MOVING")
            currentx = gpsPos[0]
            currenty = gpsPos[1]
            coordlist.append((currentx, currenty))

            if len(coordlist) > 100 and coordlist[0][0] != 0 and coordlist[0][1] != 0:  # 5 secs of data (change to 50)
                fprint("CHECKINGCORECTION")
                error = angle(
                    getAngle(coordlist[0][0], coordlist[0][1], currentx, currenty) - getAngle(currentx, currenty,
                                                                                              targetx, targety))
                fprint(str("Error: " + str(error)))
                if abs(error) > 10:
                    coordlist = []
                    fprint("ERROR - INITIATING CORRECTION")
                    if error > 0:
                        fprint("TURNING LEFT")
                        left()
                        sleep(abs(error * correctionforce))
                        forward()
                    else:
                        fprint("TURNING RIGHT")
                        right()
                        sleep(abs(error * correctionforce))
                        forward()

                else:
                    fprint("SMALL ERROR - IGNORING")
                    coordlist.pop(0)
                    forward()
            else:
                if coordlist[0][0] == 0 and coordlist[0][1] == 0 and len(coordlist) > 0:
                    coordlist.pop(0)
                    fprint("GPS ERROR")
                else:
                    fprint("NOT ENOUGH DATA")
            sleep(0.1)

        sleep(0.1)

    fprint("REACHED POINT")
    stop()


# some MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47


def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if value > 32768:
        value = value - 65536
    return value


import smbus

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address
MPU_Init()


# monitors

def MonitorIMU():
    global calib
    global imuAngle
    global gyrototal

    while True:
        gyrototal = 0
        print("CALIBRATING...")
        drift = 0
        for i in range(0, 100):
            sleep(0.05)
            gyro_z = read_raw_data(GYRO_ZOUT_H)

            # Full scale range +/- 250 degree/C as per sensitivity scale factor
            Gz = gyro_z / 131.0
            drift += Gz

        drift = drift / 100

        starttime = time()

        calib = True

        while calib:
            try:
                gyro_z = read_raw_data(GYRO_ZOUT_H)

                # Full scale range +/- 250 degree/C as per sensitivity scale factor
                Gz = gyro_z / 131.0

                gyrototal -= ((Gz - drift) / 20) * 10

                imuAngle = bigangle(gyrototal)

                sleep(0.05 - ((time() - starttime) % 0.05))
            except Exception as e:
                fprint("Error in IUM monitor loop")
                fprint(e)
                print(e)


imuThread = threading.Thread(target=MonitorIMU, name='imuThread')
imuThread.start()

#from gps import *
from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE

def MonitorGPS():
    global gpsPos
    gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)
    while True:
        try:
            report = gpsd.next()
            # print(report['class'])
            if report['class'] == 'TPV':
                gpsPos = (round(getattr(report, 'lon', 0.0), 6), round(getattr(report, 'lat', 0.0), 6))
            sleep(0.1)
        except Exception as e:
            fprint("Error in GPS loop")
            fprint(e)
            print(e)


if ExternalGPS:
    gpsThread = threading.Thread(target=MonitorGPS, name='gpsThread')
    gpsThread.start()


# nav
def Navigate():
    print("Nav thread starting")
    global targetPoints
    global isNavigating
    while True:
        print("Nav?")
        print(isNavigating)
        print("Points?")
        print(len(targetPoints))
        if isNavigating and len(targetPoints) != 0:
            fprint("Engaging Nav")
            NavigateToPoint()
            deletePoint()
        sleep(0.1)


NavThread = threading.Thread(target=Navigate, name='NavThread')
NavThread.start()


def MonitorFishFinder():
    global gpsPos
    global depth
    fprint("Connecting to fish finder...")
    ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=1.0)
    #sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
    fprint("Connection established!")
    while True:
        try:
            line = ser.readline()
            msg = pynmea2.parse(line)
            if msg.sentence_type == "GGA":
                print("We have a GPS sentence!")
                print(("Lat: " + str(msg.latitude)))
                print(("Lon: " + str(msg.longitude)))

                gpsPos = (round(msg.longitude), round(msg.latitude))

            if msg.sentence_type == "DPT":
                print("We have a depth sentence!")
                print(("Depth: " + str(msg.depth)))
                depth = depth

            # print(str(msg))

            # set depth
            # set gps

        except Exception as e:
            # fprint("Error in reading from serial fish finder")
            # fprint(e)
            print(e)

            # we silenced the errors cause checksums be dumb


if FishFinder:
    FishFinderThread = threading.Thread(target=MonitorFishFinder, name='FishFinderThread')
    FishFinderThread.start()

rightD = 4
# directon
rightP = 17
# power
leftD = 27
# direction
leftP = 22
# power
waterjet = 23

sleep(30)

print("setting up GPIO")

GPIO.setmode(GPIO.BCM)
GPIO.setup(rightD, GPIO.OUT)
GPIO.setup(rightP, GPIO.OUT)
GPIO.setup(leftD, GPIO.OUT)
GPIO.setup(leftP, GPIO.OUT)
GPIO.setup(waterjet, GPIO.OUT)

print("GPIO setup complete!")
lasttime = dt.now().minute


def timeout():
    global lasttime
    print("thread has started")
    while True:
        print(dt.now().minute)
        print("________________")
        print(lasttime)
        sleep(3)
        if dt.now().minute - lasttime > 1:
            print("autoshutoff!")
            GPIO.output(rightP, GPIO.HIGH)
            GPIO.output(leftP, GPIO.HIGH)
            GPIO.output(waterjet, GPIO.HIGH)
            lasttime = dt.now().minute


shutoff = threading.Thread(target=timeout, name='shutoff')
shutoff.start()

print("Putting pins in default state")

GPIO.output(rightP, GPIO.HIGH)
GPIO.output(leftP, GPIO.HIGH)
GPIO.output(waterjet, GPIO.HIGH)

print("motors off")

GPIO.output(rightD, GPIO.HIGH)
GPIO.output(leftD, GPIO.HIGH)

print("relays set to forward")

print("Setting up Flask")

from flask import Flask, render_template, request, jsonify

app = Flask(__name__, static_url_path='')

print("Flask is running!")

# wait for everything to set up
sleep(3)


# ACTIONS

# home page
@app.route("/")
def index():
    return render_template('index.html', name=None)


@app.route("/forward")
def forward():
    global lasttime
    lasttime = dt.now().minute
    print("Website has requested Forward")
    print("Motors off")
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    GPIO.output(waterjet, GPIO.HIGH)
    sleep(0.1)
    print("Relays to forward")
    GPIO.output(rightD, GPIO.HIGH)
    GPIO.output(leftD, GPIO.HIGH)
    sleep(0.1)
    print("Motors on")
    GPIO.output(rightP, GPIO.LOW)
    GPIO.output(leftP, GPIO.LOW)
    sleep(0.1)
    return "ok"


@app.route("/backward")
def backward():
    global lasttime
    lasttime = dt.now().minute
    print("Website has requested Backward")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    print("Relays to backward")
    GPIO.output(rightD, GPIO.LOW)
    GPIO.output(leftD, GPIO.LOW)
    sleep(0.1)
    print("Motors on")
    GPIO.output(rightP, GPIO.LOW)
    GPIO.output(leftP, GPIO.LOW)
    sleep(0.1)
    return "ok"


@app.route("/left")
def left():
    global lasttime
    lasttime = dt.now().minute
    print("Website has requested Left")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    print("Relays: Left to forward, right to backward")
    GPIO.output(rightD, GPIO.LOW)
    GPIO.output(leftD, GPIO.HIGH)
    sleep(0.1)
    print("Motors on")
    GPIO.output(rightP, GPIO.LOW)
    GPIO.output(leftP, GPIO.LOW)
    sleep(0.1)
    return "ok"


@app.route("/right")
def right():
    global lasttime
    lasttime = dt.now().minute
    print("Website has requested Right")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    print("Relays: Left to backward, right to forward")
    GPIO.output(rightD, GPIO.HIGH)
    GPIO.output(leftD, GPIO.LOW)
    sleep(0.1)
    print("Motors on")
    GPIO.output(rightP, GPIO.LOW)
    GPIO.output(leftP, GPIO.LOW)
    sleep(0.1)
    return "ok"


@app.route("/ltforward")
def ltforward():
    global lasttime
    lasttime = dt.now().minute
    print("Website has requested Left Forward")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    print("Relays: Left to forward, right to forward")
    GPIO.output(rightD, GPIO.HIGH)
    GPIO.output(leftD, GPIO.HIGH)
    sleep(0.1)
    print("Left Motor on")
    GPIO.output(leftP, GPIO.LOW)
    sleep(0.1)
    return "ok"


@app.route("/ltmini")
def ltmini():
    global lasttime
    lasttime = dt.now().minute
    print("Website has requested Left Mini")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    print("Relays: Left to forward, right to forward")
    GPIO.output(rightD, GPIO.HIGH)
    GPIO.output(leftD, GPIO.HIGH)
    sleep(0.1)
    print("Left Motor on")
    GPIO.output(leftP, GPIO.LOW)
    sleep(0.5)
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    return "ok"


@app.route("/rtforward")
def rtforward():
    global lasttime
    lasttime = dt.now().minute
    print("Website has requested Right Forward")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    print("Relays: Left to forward, right to forward")
    GPIO.output(rightD, GPIO.HIGH)
    GPIO.output(leftD, GPIO.HIGH)
    sleep(0.1)
    print("Right Motor on")
    GPIO.output(rightP, GPIO.LOW)
    sleep(0.1)
    return "ok"


@app.route("/rtmini")
def rtmini():
    global lasttime
    lasttime = dt.now().minute
    print("Website has requested Right Mini")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    print("Relays: Left to forward, right to forward")
    GPIO.output(rightD, GPIO.HIGH)
    GPIO.output(leftD, GPIO.HIGH)
    sleep(0.1)
    print("Right Motor on")
    GPIO.output(rightP, GPIO.LOW)
    sleep(0.5)
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    return "ok"


@app.route("/stop")
def stop():
    print("Website has requested STOP")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    return "ok"


@app.route("/jet")
def jet():
    print("Website has requested water JET")
    print("Motors off")
    GPIO.output(waterjet, GPIO.HIGH)
    GPIO.output(rightP, GPIO.HIGH)
    GPIO.output(leftP, GPIO.HIGH)
    sleep(0.1)
    print("water jet active")
    GPIO.output(waterjet, GPIO.LOW)
    sleep(0.1)
    return "ok"


@app.route("/loc")
def loc():
    global imuAngle
    global gpsPos
    global cdist
    global distance
    global depth
    print("Website has requested location + rotation")
    data = {"Rotation": imuAngle, "GPSX": gpsPos[1], "GPSY": gpsPos[0], "Dist": cdist, "TDist": distance,
            "Depth": depth}
    return jsonify(data)


@app.route("/addpoint")
def newPoint():
    fprint("NEW POINT!")
    global isNavigating
    x = float(request.args['x'])
    y = float(request.args['y'])
    isNavigating = True  # because we don't want the thread to instantly terminate
    addPoint(y, x)  # is dumb cause gps
    isNavigating = True
    return "ok"


@app.route("/overide")
def overide():
    global isNavigating
    global targetPoints
    fprint("OVERIDE")
    isNavigating = False
    targetPoints = []
    stop()
    return "ok"


@app.route("/calib")
def Calib():
    global calib
    fprint("CALIBRATE IMU")
    calib = False
    return "ok"


@app.route("/reset")
def resetIMU():
    global imuAngle
    global gyrototal
    fprint("RESET")
    imuAngle = 0
    gyrototal = 0
    return "ok"


@app.route("/terminal")
def terminal():
    global printList
    fulldata = "<html><body><p>"
    for item in printList:
        fulldata = fulldata + str(item) + "<br>"
    fulldata = fulldata + "</p></body></html>"
    return fulldata


@app.route("/settings")
def viewVars():
    global correctionforce
    global distance

    return "<html><body><p>cforce: " + str(correctionforce) + "<br>Distance : " + str(distance) + "</p></body></html>"


@app.route("/setvar")
def setvar():
    global correctionforce
    global distance
    var = str(request.args['var'])
    value = float(request.args['value'])

    if var == "cforce":
        correctionforce = value
        return "Succesfully updated correction force to " + str(value)
    elif var == "dist":
        distance = value
        return "Succesfully updated distance to " + str(value)
    else:
        return "OOPS! We couldn't find that variable! Try cforce or dist!"


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=80, debug=True)
