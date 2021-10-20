import math
import time
from time import sleep
#MATH FUNCTIONS

# function that converts radians (0 - 2pi) into degrees (0 - 360)
def degrees(radians):
    return (radians / math.pi) * 180

# converts to negative angle format (-180 to 180)
def angle(x):
    x = x % 360
    if x > 180:
        x -= 360
    return x

# converts to positive angle only format (0 to 360)
def bigangle(x):
    x = x % 360
    x += 360
    x = x % 360
    return x


#function that returns the angle off of north for the given points
def getAngle(x1, y1, x2, y2):
    return degrees(math.atan2((x2 - x1), (y2 - y1)))


#gets the distance between two points
def getDistance(x1, y1, x2, y2):
    return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

class DroneBoatGPS:
    def __init__(self, droneBoat):
        self.droneBoat = droneBoat
        self.targetPoints = []
        self.printList = []

        self.isNavigating = False
        self.calib = False
        self.gpsPos = (0, 0)  # this is dumb and is in X,Y (W,N) notation
        self.imuAngle = 0
        self.cdist = 0
        self.depth = 0
        self.gyrototal = 0
        self.IMURecalib = 1

        # nav config
        self.correctionforce = 0.01
        self.distance = 0.0001
        self.coordList = []


    def fprint(self, message):
        print(message)  # for terminal users...
        self.printList.append(message)

    def addPoint(self, x, y):
        self.fprint(("ADDING A POINT! " + str(x) + " " + str(y)))
        self.targetPoints.append((x, y))

    def deletePoint(self):
        if len(self.targetPoints) > 0:
            return self.targetPoints.pop(0)
        else:
            self.fprint("EMPTY LIST, you probably did an overide?")
            return None

    def pointAtAngle(self, target, accuracy):
        print(("Target " + str(target)))
        self.droneBoat.lasttime = time.time()
        if bigangle(bigangle(target) - bigangle(self.imuAngle)) <= 180:
            self.fprint("RIGHT TURN")
            self.droneBoat.right()
        else:
            self.fprint("LEFT TURN")
            self.droneBoat.left()

        while abs(bigangle(target) - bigangle(self.imuAngle)) > accuracy and self.isNavigating:
            print("TURNING!")

        self.droneBoat.stop()
        self.fprint("DONE!")

    def navigateToPoint(self):
        targetx = self.targetPoints[0][0]
        targety = self.targetPoints[0][1]

        while getDistance(targetx, targety, self.gpsPos[0], self.gpsPos[1]) > self.distance and self.isNavigating:
            self.coordList = []
            self.fprint("pointing at angle")
            while self.gpsPos[0] == 0:
                pass
            currentx = self.gpsPos[0]
            currenty = self.gpsPos[1]
            targetAngle = getAngle(currentx, currenty, targetx, targety)
            self.pointAtAngle(targetAngle, 5)

            self.droneBoat.forward()
            error = 0
            while getDistance(targetx, targety, self.gpsPos[0], self.gpsPos[1]) > self.distance and self.isNavigating and error < 45:
                self.cdist = getDistance(targetx, targety, self.gpsPos[0], self.gpsPos[1])
                self.fprint("MOVING")
                currentx = self.gpsPos[0]
                currenty = self.gpsPos[1]
                self.coordList.append((currentx, currenty))

                if len(self.coordList) > 100 and self.coordList[0][0] != 0 and self.coordList[0][1] != 0:  # 5 secs of data (change to 50)
                    self.fprint("CHECKINGCORECTION")
                    a = getAngle(self.coordList[0][0], self.coordList[0][1], currentx, currenty)
                    b = getAngle(currentx, currenty, targetx, targety)
                    error = angle(a - b)

                    if self.IMURecalib == 1:
                        x = bigangle(getAngle(self.coordList[0][0], self.coordList[0][1], currentx, currenty))
                        self.imuAngle = x
                        self.gyrototal = x

                    self.fprint(str("Error: " + str(error)))
                    if abs(error) > 10:
                        self.coordList = []
                        self.fprint("ERROR - INITIATING CORRECTION")
                        if error > 0:
                            self.fprint("TURNING LEFT")
                            self.droneBoat.left()
                            sleep(abs(error * self.correctionforce))
                            self.droneBoat.forward()
                        else:
                            self.fprint("TURNING RIGHT")
                            self.droneBoat.right()
                            sleep(abs(error * self.correctionforce))
                            self.droneBoat.forward()

                    else:
                        self.fprint("SMALL ERROR - IGNORING")
                        self.coordList.pop(0)
                        self.droneBoat.forward()
                else:
                    if self.coordList[0][0] == 0 and self.coordList[0][1] == 0 and len(self.coordList) > 0:
                        self.coordList.pop(0)
                        self.fprint("GPS ERROR")
                    else:
                        self.fprint("NOT ENOUGH DATA")
                sleep(0.1)

            sleep(0.1)

        self.fprint("REACHED POINT")
        self.droneBoat.stop()
