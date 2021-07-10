#!/urs/bin/env python
# imports
import pynmea2

import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime as dt
import threading
import os
from time import time

# for fish finder interaction
import io
import serial

# Mode
ExternalGPS = False
FishFinder = True
IMURecalib = 1  # to make it easier to set from the website
AutoShutoffTime = 5
Map = "LimeLakeFishing"

# Maps
MapData = {
    "Pond": {
        "TopN": 39.998929,
        "TopW": -86.151867,
        "BottomN": 39.998573,
        "BottomW": -86.151004,
        "MapW": 500,
        "MapH": 250,
        "Source": "pond.png",
        "Res": 0.2,
        "MaxDepth": 5
    },
    "Meadowlark": {
        "TopN": 39.983208,
        "TopW": -86.136908,
        "BottomN": 39.982026,
        "BottomW": -86.135181,
        "MapW": 500,
        "MapH": 500,
        "Source": "meadowlark.png",
        "Res": 0.2,
        "MaxDepth": 5
    },
    "LimeLakeSmall": {
        "TopN": 44.886608,
        "TopW": -85.845936,
        "BottomN": 44.884576,
        "BottomW": -85.841345,
        "MapW": 500,
        "MapH": 500,
        "Source": "LimeLakeSmall.png",
        "Res": 0.2,
        "MaxDepth": 3  # meters!
    },
    "LimeLakeFishing": {
        "TopN": 44.894234,
        "TopW": -85.852445,
        "BottomN": 44.883510,
        "BottomW": -85.830756,
        "MapW": 500,
        "MapH": 500,
        "Source": "LimeLakeFishing.png",
        "Res": 0.2,
        "MaxDepth": 10  # meters!
    }
}

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
correctionforce = 0.02
distance = 0.000075
isNavigating = False
calib = False
cdist = 0
depth = 0

targetPoints = []
coordlist = []
depthdata = []
depthjson = []
lastPoint = None

MapArray = []
PopulatedMap = []

gpsPos = (0, 0)  # this is dumb and is in X,Y (W,N) notation
imuAngle = 0
gyrototal = 0  # an unformatted version


# math functions


# function that converts radians (0 - 2pi) into degrees (0 - 360)
def degrees(radians):
    return (radians / math.pi) * 180


# converts to negative angle format
def angle(bigangle):
    bigangle = bigangle % 360
    if bigangle > 180:
        bigangle -= 360
    return bigangle


# converts to positive angle only format
def bigangle(angle):
    angle = angle % 360
    angle += 360
    angle = angle % 360
    return angle


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
    global lastPoint
    global targetPoints
    if len(targetPoints) > 0:
        lastPoint = targetPoints[0]
        fprint(targetPoints.pop(0))
    else:
        fprint("EMPTY LIST, you probably did an overide?")
    return


# depth map funcs
def GetPoint(x, y):
    global MapArray
    try:
        return MapArray[y][x]
    except:
        return None


def GetAverageTile(x, y, i):
    total = 0
    count = 0
    for ydif in range((y - i), (y + i)):
        for xdif in range((x - i), (x + i)):
            if GetPoint(xdif, ydif) != None:
                total += GetPoint(xdif, ydif)
                count += 1
    if count == 0:
        return None
    return round((float(total) / count), 6)


def PopulateTile(x, y):
    global PopulatedMap
    for i in range(0, 3):
        t = GetAverageTile(x, y, i)
        if t != None:
            PopulatedMap[y][x] = t
            return
    return


def color(cType, val):
    if cType == "red":
        val += 0
    if cType == "green":
        val += float(2) / 3
    if cType == "blue":
        val += float(1) / 3

    if val >= 1:
        val -= 1

    if val <= float(1) / 3:
        return 255 * (float(1) / 3 - val) * 3
    if val >= float(2) / 3:
        return 255 * (val - float(2) / 3) * 3

    return 0


def colorificate(dmin, dval, dmax):
    d = (dval - dmin) / (dmax - dmin)
    red = round(color("red", d))
    green = round(color("green", d))
    blue = round(color("blue", d))

    opacity = 50

    return (red, green, blue, opacity)


def creatediv(x, y, xl, yl, color):
    start = '<div id="MapPixel" style="'
    end = '"></div>'
    px = 'px; '

    return (start +
            'top: ' + str(y) + px +
            'left: ' + str(x) + px +
            'width: ' + str(xl) + px +
            'height: ' + str(yl) + px +
            'background: rgba' + str(color) + '; ' +
            'position: absolute' +
            end)


def makeJS():
    global MapData
    global Map
    m = MapData[Map]
    start = "<script>window.onload=OnLoad;\n"
    end = "</script>"
    func = (
        "function OnLoad() \n { \n //alert(navigator.geolocation); \n if(navigator.geolocation) \n { \n //alert('working!'); \n navigator.geolocation.getCurrentPosition(move);\n } \n else \n { \n //alert('NO GPS') \n } \n}\n  function move(pos) \n {\n //alert('HELLO!');\n moveMark(pos.coords.latitude, pos.coords.longitude);\n }\n")
    func2 = (
                "function moveMark(gpsN, gpsW)\n { var N = (((gpsN-gpsBX)/(gpsTX-gpsBX))*mapH) - 20; \n var W = (((gpsW-gpsBY)/(gpsTY-gpsBY))*mapW) - 10; \n  N = N.toFixed(0); \n  W = W.toFixed(0); \n var elem = document.getElementById(" + '"phone"' + "); \n //alert(elem); \n //alert(N); \n elem.style.top = N + 'px'; \n  elem.style.left = W + 'px';\n elem.style.position = 'absolute';\n }\n")
    html = '<img id="phone" src="markerPhone.png" height=20 width=20>'
    vardef = ('var mapW = ' + str(m["MapW"]) + ';\n' +
              'var mapH = ' + str(m["MapH"]) + ';\n' +
              'var gpsBX = ' + str(m["TopN"]) + ';\n' +
              'var gpsBY = ' + str(m["TopW"]) + ';\n' +
              'var gpsTX = ' + str(m["BottomN"]) + ';\n' +
              'var gpsTY = ' + str(m["BottomW"]) + ';\n')
    return html + start + vardef + func + func2 + end


def PlaceOnMap(MapTopY, MapTopX, MapBottomY, MapBottomX, MapH, MapW, X, Y, Depth):
    global MapArray
    truex = int(round(((X - MapTopX) / (MapBottomX - MapTopX)) * MapW))
    truey = int(round(((Y - MapTopY) / (MapBottomY - MapTopY)) * MapH))
    try:
        MapArray[truey][truex] = Depth
    except Exception as e:
        fprint("Can't place value on map! Error in Loc y/Map y, Loc x, Map x")
        fprint(e)
        fprint((str(truey) + " " + str(len(MapArray))))
        fprint((str(truex) + " " + str(len(MapArray[0]))))
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
    global gyrototal
    global distance
    targetx = targetPoints[0][0]
    targety = targetPoints[0][1]
    error = 0
    while getDistance(targetx, targety, gpsPos[0], gpsPos[1]) > distance and isNavigating:
        coordlist = []
        fprint("pointing at angle")
        while gpsPos[0] == 0:
            pass
        currentx = gpsPos[0]
        currenty = gpsPos[1]
        targetAngle = getAngle(currentx, currenty, targetx, targety)
        pointAtAngle(targetAngle, 5)

        sleep(0.05)
        forward()
        error = 0
        while getDistance(targetx, targety, gpsPos[0], gpsPos[1]) > distance and isNavigating and error < 45:
            cdist = getDistance(targetx, targety, gpsPos[0], gpsPos[1])
            # fprint("MOVING")
            currentx = gpsPos[0]
            currenty = gpsPos[1]
            coordlist.append((currentx, currenty))

            if len(coordlist) > 100 and coordlist[0][0] != 0 and coordlist[0][1] != 0:  # 5 secs of data (change to 50)
                fprint("CHECKINGCORECTION")
                error = angle(
                    getAngle(coordlist[0][0], coordlist[0][1], currentx, currenty) - getAngle(currentx, currenty,
                                                                                              targetx, targety))

                if IMURecalib == 1:
                    imuAngle = bigangle(getAngle(coordlist[0][0], coordlist[0][1], currentx, currenty))
                    gyrototal = bigangle(getAngle(coordlist[0][0], coordlist[0][1], currentx, currenty))

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
                    # forward() we don't need this - it spams the relays
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

gyrototal = 0


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
    if (value > 32768):
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

        starttime = time.time()

        calib = True

        while calib:
            try:
                gyro_z = read_raw_data(GYRO_ZOUT_H)

                # Full scale range +/- 250 degree/C as per sensitivity scale factor
                Gz = gyro_z / 131.0

                gyrototal -= ((Gz - drift) / 20) * 10

                imuAngle = bigangle(gyrototal)

                sleep(0.05 - ((time.time() - starttime) % (0.05)))
            except Exception as e:
                fprint("Error in IUM monitor loop")
                fprint(e)
                print(e)


imuThread = threading.Thread(target=MonitorIMU, name='imuThread')
imuThread.start()

from gps import *


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
    fprint("Responsibly starting gps")
    gpsThread = threading.Thread(target=MonitorGPS, name='gpsThread')
    gpsThread.start()


# nav
def Navigate():
    print("Nav thread starting")
    global targetPoints
    global isNavigating
    while True:
        # print("Nav?")
        # print(isNavigating)
        # print("Points?")
        # print(len(targetPoints))
        if isNavigating and len(targetPoints) != 0:
            fprint("Engaging Nav")
            NavigateToPoint()
            deletePoint()
        sleep(0.1)


NavThread = threading.Thread(target=Navigate, name='NavThread')
NavThread.start()


def MonitorFishFinder():
    global depthdata
    global gpsPos
    global depth
    fprint("Connecting to fish finder...")
    ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=1.0)
    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
    fprint("Connection established!")
    while True:
        try:
            line = ser.readline()
            msg = pynmea2.parse(line)
            if msg.sentence_type == "GGA":
                # print("We have a GPS sentence!")
                # print(("Lat: " + str(msg.latitude)))
                # print(("Lon: " + str(msg.longitude)))

                gpsPos = (round(msg.longitude, 6), round(msg.latitude, 6))

                line = "GPSN: " + str(gpsPos[1]) + " GPSW: " + str(gpsPos[0]) + " DPT: " + str(depth) + ",\n"
                jsonLine = {
                    "GPSN": str(gpsPos[1]),
                    "GPSW": str(gpsPos[0]),
                    "DPT": str(depth)
                }

                depthdata.append(line)
                depthjson.append(jsonLine)
            if msg.sentence_type == "DPT":
                # print("We have a depth sentence!")
                # print(("Depth: " + str(msg.depth)))
                depth = round(msg.depth, 6)

            # print(str(msg))

            # set depth
            # set gps

        except Exception as e:
            # print("Error in reading from serial fish finder")
            # print(e)
            # print(e)
            pass
            # we silenced the errors cause checksums be dumb


# we can't start the thread here, because flask is dumb
if FishFinder:
    fprint("Responsibly starting fish")
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

sleep(10)

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
    global AutoShutoffTime
    print("thread has started")
    while True:
        # print(dt.now().minute)
        # print("________________")
        # print(lasttime)
        sleep(3)
        if dt.now().minute - lasttime >= AutoShutoffTime:
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
    lastime = dt.now().minute
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
    lastime = dt.now().minute
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
    global targetPoints
    global lastPoint
    global printList
    print("Website has requested location + rotation")

    truedepth = (depth * 3) + 1  # convert to feet, add 1 to compensate for sensor loc

    data = {}
    data["Rotation"] = imuAngle
    data["GPSX"] = gpsPos[1]
    data["GPSY"] = gpsPos[0]
    data["Dist"] = cdist
    data["TDist"] = distance
    data["Depth"] = truedepth
    if (lastPoint != None):
        data["OLDX"] = lastPoint[1]
        data["OLDY"] = lastPoint[0]
    data["Print"] = printList[(len(printList) - 1)]

    points = []
    for point in targetPoints:
        pointData = {}
        pointData["GPSX"] = point[0]
        pointData["GPSY"] = point[1]
        points.append(pointData)

    data["TargetPoints"] = points

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


@app.route("/scanarea")
def scanArea():
    fprint("AREA SCAN REQUESTED")
    global isNavigating
    tx = float(request.args['tx'])
    ty = float(request.args['ty'])
    bx = float(request.args['bx'])
    by = float(request.args['by'])
    LineCount = int(request.args['lines'])

    XList = [tx, bx]
    frac = (float(1) / LineCount)
    YList = []
    for i in range(0, (LineCount + 1)):
        YList.append(frac * i * (by - ty) + ty)

    side = 0
    for y in YList:
        x = XList[(side % 2)]
        isNavigating = True
        addPoint(y, x)
        side += 1

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
def calib():
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
    fprint("----------------------------")
    fulldata = "<html><body><p>"
    for item in printList:
        if (item != "NOT ENOUGH DATA"):
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
    global IMURecalib
    global AutoShutoffTime
    global Map
    var = str(request.args['var'])

    if var == "map":
        Map = str(request.args['value'])
        return str("Map is now: " + str(Map))

    value = float(request.args['value'])

    if var == "cforce":
        correctionforce = value
        return str("Succesfully updated correction force to " + str(value))
    elif var == "dist":
        distance = value
        return str("Succesfully updated distance to " + str(value))
    elif var == "imurecalib":
        IMURecalib = value
        return str("Succesfully updated imurecalib to " + str(value))
    elif var == "shutofftime":
        AutoShutoffTime = value
        return str("Succesfully updated shutofftime to " + str(value))
    else:
        return "OOPS! We couldn't find that variable! Try cforce. imurecalib, shutofftime or dist!"


@app.route("/map")
def map():
    global MapData
    global Map
    return jsonify(MapData[Map])


@app.route("/dumpdata")
def writeToFile():
    global depthdata
    fname = str(request.args["name"]) + ".txt"
    f = open(fname, "w+")
    for line in depthdata:
        f.write(line)
    f.close()
    return "File written."


@app.route("/cleardata")
def writeToFileFlask():
    global depthdata
    global depthjson
    depthjson = []
    depthdata = []
    return "Data cleared."


@app.route("/depthmap")
def depthMap():
    global depthjson
    global MapData
    global Map
    global MapArray
    global PopulatedMap

    MapArray = []
    PopulatedMap = []

    myMap = MapData[Map]
    shrink = float(1) / myMap["Res"]
    scale = myMap["Res"]

    for x in range(0, int(round(myMap["MapH"] * scale))):
        tempmap = []
        for y in range(0, int(round(myMap["MapW"] * scale))):
            tempmap.append(None)
        MapArray.append(tempmap)

    for x in range(0, int(round(myMap["MapH"] * scale))):
        tempmap = []
        for y in range(0, int(round(myMap["MapW"]))):
            tempmap.append(None)
        PopulatedMap.append(tempmap)

    for line in depthjson:
        PlaceOnMap(float(myMap["TopN"]), float(myMap["TopW"]), float(myMap["BottomN"]), float(myMap["BottomW"]),
                   (float(myMap["MapH"]) * scale), (float(myMap["MapW"]) * scale), float(line["GPSW"]),
                   float(line["GPSN"]), float(line["DPT"]))

    for y in range(0, len(MapArray)):
        for x in range(0, len(MapArray[y])):
            PopulateTile(x, y)

    MapArray = PopulatedMap
    start = '<!DOCTYPE html><html><body><img src='
    more = ' height="'
    evenmore = '" width="'
    stillmore = '">'
    end = "</body></html>"
    divlist = []
    for i in range(0, len(MapArray)):
        for z in range(0, len(MapArray[i])):
            try:
                divlist.append(creatediv((z * shrink), (i * shrink), shrink, shrink,
                                         colorificate(0, MapArray[i][z], myMap["MaxDepth"])))
            except:
                pass  # fail quietly
    fullstart = start + myMap["Source"] + more + str(myMap["MapH"]) + evenmore + str(myMap["MapW"]) + stillmore
    full = fullstart
    for div in divlist:
        full = full + div
    full = full + makeJS()
    full = full + end
    return full


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=80, debug=True, use_reloader=False)
