import smbus
import serial
import pynmea2
from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE
from time import sleep, time
from DroneBoatGPS import bigangle

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

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

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
        value -= 65536
    return value



def MonitorIMU(droneBoatGPS):
    while True:
        droneBoatGPS.gyrototal = 0
        print("CALIBRATING...")
        drift = 0
        for i in range(0, 100):
            sleep(0.05)
            gyro_z = read_raw_data(GYRO_ZOUT_H)

            # Full scale range +/- 250 degree/C as per sensitivity scale factor
            Gz = gyro_z / 131.0
            drift += Gz

        drift /= 100

        starttime = time()

        droneBoatGPS.calib = True

        while droneBoatGPS.calib:
            try:
                gyro_z = read_raw_data(GYRO_ZOUT_H)

                # Full scale range +/- 250 degree/C as per sensitivity scale factor
                Gz = gyro_z / 131.0

                droneBoatGPS.gyrototal -= ((Gz - drift) / 20) * 10

                droneBoatGPS.imuAngle = bigangle(droneBoatGPS.gyrototal)

                sleep(0.05 - ((time() - starttime) % 0.05))
            except Exception as e:
                droneBoatGPS.fprint("Error in IUM monitor loop")
                droneBoatGPS.fprint(e)
                print(e)


def MonitorGPS(droneBoatGPS):
    gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)
    while True:
        try:
            report = gpsd.next()
            # print(report['class'])
            if report['class'] == 'TPV':
                droneBoatGPS.gpsPos = (round(getattr(report, 'lon', 0.0), 6), round(getattr(report, 'lat', 0.0), 6))
            sleep(0.1)
        except Exception as e:
            droneBoatGPS.fprint("Error in GPS loop")
            droneBoatGPS.fprint(e)
            print(e)


def MonitorFishFinder(droneBoatGPS, depthMap):
    droneBoatGPS.fprint("Connecting to fish finder...")
    ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=1.0)
    droneBoatGPS.fprint("Connection established!")
    while True:
        try:
            line = ser.readline()
            msg = pynmea2.parse(line)
            if msg.sentence_type == "GGA":
                print("We have a GPS sentence!")
                print(("Lat: " + str(msg.latitude)))
                print(("Lon: " + str(msg.longitude)))

                droneBoatGPS.gpsPos = (round(msg.longitude), round(msg.latitude))

                line = "GPSN: " + str(droneBoatGPS.gpsPos[1]) + " GPSW: " + str(
                    droneBoatGPS.gpsPos[0]) + " DPT: " + str(droneBoatGPS.depth) + ",\n"
                jsonLine = {
                    "GPSN": str(droneBoatGPS.gpsPos[1]),
                    "GPSW": str(droneBoatGPS.gpsPos[0]),
                    "DPT": str(droneBoatGPS.depth)
                }

                depthMap.depthdata.append(line)
                depthMap.depthjson.append(jsonLine)

            if msg.sentence_type == "DPT":
                print("We have a depth sentence!")
                print(("Depth: " + str(msg.depth)))
                droneBoatGPS.depth = msg.depth

        except (Exception,):
            # we silenced the errors cause checksums be dumb
            pass
