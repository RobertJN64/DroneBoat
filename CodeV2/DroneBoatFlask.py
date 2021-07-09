from flask import Flask, render_template, request, jsonify
from DroneBoat import DroneBoat
from DroneBoatGPS import DroneBoatGPS
import DroneBoatIO
from time import sleep
import threading

#CONFIG
ExternalGPS = False
FishFinder = True

sleep(10)
droneBoat = DroneBoat()
droneBoatGPS = DroneBoatGPS(droneBoat)


droneBoatGPS.fprint("DroneBoatMain python script started!")
if ExternalGPS:
    droneBoatGPS.fprint("We are running with an external GPS plugged in!")
if FishFinder:
    droneBoatGPS.fprint("We are running with a fish finder plugged in!")

app = Flask(__name__, static_url_path='')

@app.route("/")
def index():
    return render_template('index.html', name=None)

@app.route("/forward")
def forward():
    print("Website has requested forward")
    droneBoat.forward()
    return "ok"

@app.route("/backward")
def backward():
    print("Website has requested backward")
    droneBoat.backward()
    return "ok"

@app.route("/left")
def left():
    print("Website has requested left")
    droneBoat.left()
    return "ok"

@app.route("/right")
def right():
    print("Website has requested right")
    droneBoat.right()
    return "ok"

@app.route("/ltforward")
def ltforward():
    print("Website has requested left forward")
    droneBoat.leftForward()
    return "ok"

@app.route("/ltmini")
def ltmini():
    print("Website has requested left mini")
    droneBoat.leftMini()
    return "ok"

@app.route("/rtforward")
def rtforward():
    print("Website has requested right forward")
    droneBoat.rightForward()
    return "ok"

@app.route("/rtmini")
def rtmini():
    print("Website has requested right mini")
    droneBoat.rightMini()
    return "ok"

@app.route("/stop")
def stop():
    print("Website has requested stop")
    droneBoat.stop()
    return "ok"

@app.route("/jet")
def jet():
    print("Website has requested aux")
    droneBoat.aux()
    return "ok"


@app.route("/addpoint")
def newPoint():
    droneBoatGPS.fprint("NEW POINT!")
    x = float(request.args['x'])
    y = float(request.args['y'])
    droneBoatGPS.isNavigating = True  # because we don't want the thread to instantly terminate
    droneBoatGPS.addPoint(y, x)  # is dumb cause gps
    droneBoatGPS.isNavigating = True
    return "ok"


@app.route("/settings")
def viewVars():
    return ("<html><body><p>cforce: " + str(droneBoatGPS.correctionforce) +
            "<br>Distance : " + str(droneBoatGPS.distance) + "</p></body></html>")

@app.route("/setvar")
def setvar():
    var = str(request.args['var'])
    value = float(request.args['value'])

    if var == "cforce":
        droneBoatGPS.correctionforce = value
        return "Succesfully updated correction force to " + str(value)
    elif var == "dist":
        droneBoatGPS.distance = value
        return "Succesfully updated distance to " + str(value)
    else:
        return "OOPS! We couldn't find that variable! Try cforce or dist!"


@app.route("/loc")
def loc():
    print("Website has requested location + rotation")
    data = {"Rotation": droneBoatGPS.imuAngle, "GPSX": droneBoatGPS.gpsPos[1], "GPSY": droneBoatGPS.gpsPos[0],
            "Dist": droneBoatGPS.cdist, "TDist": droneBoatGPS.distance, "Depth": droneBoatGPS.depth}
    return jsonify(data)


@app.route("/overide")
def overide():
    droneBoatGPS.fprint("OVERIDE")
    droneBoatGPS.isNavigating = False
    droneBoatGPS.targetPoints = []
    droneBoat.stop()
    return "ok"

@app.route("/terminal")
def terminal():
    fulldata = "<html><body><p>"
    for item in droneBoatGPS.printList:
        fulldata = fulldata + str(item) + "<br>"
    fulldata = fulldata + "</p></body></html>"
    return fulldata


@app.route("/calib")
def Calib():
    droneBoatGPS.fprint("CALIBRATE IMU")
    droneBoatGPS.calib = False
    return "ok"


@app.route("/reset")
def resetIMU():
    droneBoatGPS.fprint("RESET")
    droneBoatGPS.imuAngle = 0
    droneBoatGPS.gyrototal = 0
    return "ok"

def timeoutThread():
    print("Timeout thread starting")
    while True:
        sleep(3)
        droneBoat.timeout()

def Navigate():
    print("Nav thread starting")
    while True:
        if droneBoatGPS.isNavigating and len(droneBoatGPS.targetPoints) != 0:
            droneBoatGPS.fprint("Engaging Nav")
            droneBoatGPS.navigateToPoint()
            droneBoatGPS.deletePoint()
        sleep(0.1)


shutoffThread = threading.Thread(target=timeoutThread, name='shutoff')
shutoffThread.start()

DroneBoatIO.MPU_Init()
imuThread = threading.Thread(target=DroneBoatIO.MonitorIMU, args=(droneBoatGPS,), name='imuThread')
imuThread.start()

NavThread = threading.Thread(target=Navigate, name='NavThread')
NavThread.start()

if ExternalGPS:
    gpsThread = threading.Thread(target=DroneBoatIO.MonitorGPS, args=(droneBoatGPS,), name='gpsThread')
    gpsThread.start()

if FishFinder:
    FishFinderThread = threading.Thread(target=DroneBoatIO.MonitorFishFinder, args=(droneBoatGPS,), name='FishFinderThread')
    FishFinderThread.start()

print("Flask is running!")
app.run(host='0.0.0.0', port=80, debug=True)