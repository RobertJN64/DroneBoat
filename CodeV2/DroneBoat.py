from time import sleep
import RPi.GPIO as GPIO
import time


class DroneBoat:
    def __init__(self):
        # Config
        self.rightD = 4  # Direction
        self.rightP = 17  # Power
        self.leftD = 27  # Direction
        self.leftP = 22  # Power
        self.waterjet = 23  # Aux power

        print("setting up GPIO")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.rightD, GPIO.OUT)
        GPIO.setup(self.rightP, GPIO.OUT)
        GPIO.setup(self.leftD, GPIO.OUT)
        GPIO.setup(self.leftP, GPIO.OUT)
        GPIO.setup(self.waterjet, GPIO.OUT)

        print("GPIO setup complete!")

        print("Putting pins in default state")

        GPIO.output(self.rightP, GPIO.HIGH)
        GPIO.output(self.leftP, GPIO.HIGH)
        GPIO.output(self.waterjet, GPIO.HIGH)

        print("motors off")

        GPIO.output(self.rightD, GPIO.HIGH)
        GPIO.output(self.leftD, GPIO.HIGH)

        print("relays set to forward")

        self.lasttime = time.time()

    def timeout(self):
        if time.time() - self.lasttime > 5 * 60:
            print("autoshutoff!")
            self.motorsOff()
            self.lasttime = time.time()

    def motorsOff(self):
        print("Motors off")
        GPIO.output(self.rightP, GPIO.HIGH)
        GPIO.output(self.leftP, GPIO.HIGH)
        GPIO.output(self.waterjet, GPIO.HIGH)

    def forward(self):
        self.lasttime = time.time()
        self.motorsOff()
        sleep(0.1)
        print("Relays to forward")
        GPIO.output(self.rightD, GPIO.HIGH)
        GPIO.output(self.leftD, GPIO.HIGH)
        sleep(0.1)
        print("Motors on")
        GPIO.output(self.rightP, GPIO.LOW)
        GPIO.output(self.leftP, GPIO.LOW)
        sleep(0.1)

    def backward(self):
        self.lasttime = time.time()
        self.motorsOff()
        sleep(0.1)
        print("Relays to backward")
        GPIO.output(self.rightD, GPIO.LOW)
        GPIO.output(self.leftD, GPIO.LOW)
        sleep(0.1)
        print("Motors on")
        GPIO.output(self.rightP, GPIO.LOW)
        GPIO.output(self.leftP, GPIO.LOW)
        sleep(0.1)

    def left(self):
        self.lasttime = time.time()
        self.motorsOff()
        sleep(0.1)
        print("Relays: Left to forward, right to backward")
        GPIO.output(self.rightD, GPIO.LOW)
        GPIO.output(self.leftD, GPIO.HIGH)
        sleep(0.1)
        print("Motors on")
        GPIO.output(self.rightP, GPIO.LOW)
        GPIO.output(self.leftP, GPIO.LOW)
        sleep(0.1)

    def right(self):
        self.lasttime = time.time()
        self.motorsOff()
        sleep(0.1)
        print("Relays: Left to backward, right to forward")
        GPIO.output(self.rightD, GPIO.HIGH)
        GPIO.output(self.leftD, GPIO.LOW)
        sleep(0.1)
        print("Motors on")
        GPIO.output(self.rightP, GPIO.LOW)
        GPIO.output(self.leftP, GPIO.LOW)
        sleep(0.1)

    def leftForward(self):
        self.lasttime = time.time()
        self.motorsOff()
        sleep(0.1)
        print("Relays: Left to forward, right to forward")
        GPIO.output(self.rightD, GPIO.HIGH)
        GPIO.output(self.leftD, GPIO.HIGH)
        sleep(0.1)
        print("Left Motor on")
        GPIO.output(self.leftP, GPIO.LOW)
        sleep(0.1)

    def leftMini(self):
        self.lasttime = time.time()
        self.motorsOff()
        sleep(0.1)
        print("Relays: Left to forward, right to forward")
        GPIO.output(self.rightD, GPIO.HIGH)
        GPIO.output(self.leftD, GPIO.HIGH)
        sleep(0.1)
        print("Left Motor on")
        GPIO.output(self.leftP, GPIO.LOW)
        sleep(0.5)
        self.motorsOff()
        sleep(0.1)

    def rightForward(self):
        self.lasttime = time.time()
        self.motorsOff()
        sleep(0.1)
        print("Relays: Left to forward, right to forward")
        GPIO.output(self.rightD, GPIO.HIGH)
        GPIO.output(self.leftD, GPIO.HIGH)
        sleep(0.1)
        print("Right Motor on")
        GPIO.output(self.rightP, GPIO.LOW)
        sleep(0.1)

    def rightMini(self):
        self.lasttime = time.time()
        self.motorsOff()
        sleep(0.1)
        print("Relays: Left to forward, right to forward")
        GPIO.output(self.rightD, GPIO.HIGH)
        GPIO.output(self.leftD, GPIO.HIGH)
        sleep(0.1)
        print("Right Motor on")
        GPIO.output(self.rightP, GPIO.LOW)
        sleep(0.5)
        self.motorsOff()
        sleep(0.1)

    def stop(self):
        self.motorsOff()
        sleep(0.1)

    def aux(self):
        self.motorsOff()
        sleep(0.1)
        print("water jet active")
        GPIO.output(self.waterjet, GPIO.LOW)
        sleep(0.1)
