import argparse
import time
import math

# Argument parsing
parser = argparse.ArgumentParser(description='MPU6050 Data Reader')
parser.add_argument('--sim', action='store_true', help='Run in simulation mode')
args = parser.parse_args()

# Conditional imports and mocking for simulation
if args.sim:
    from unittest.mock import MagicMock
    import sys

    # Mocking RPi.GPIO and smbus for simulation
    sys.modules['RPi'] = MagicMock()
    sys.modules['RPi.GPIO'] = MagicMock()
    sys.modules['smbus'] = MagicMock()

    import smbus
    import serial

    # Mocking smbus
    bus = smbus.SMBus(1)
    bus.write_byte_data = MagicMock()
    bus.read_byte_data = MagicMock(return_value=0)

    # Mocking serial
    SERIAL_PORT = "/dev/ttyS0"
    ser = MagicMock()

else:
    import smbus
    import serial

    SERIAL_PORT = "/dev/ttyS0"
    ser = serial.Serial(SERIAL_PORT, baudrate=9600, timeout=5)

# MPU6050 Registers and their Address
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

# Initialize MPU6050
def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

# Read raw data from MPU6050
def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

Device_Address = 0x68  # MPU6050 device address

if not args.sim:
    MPU_Init()

print("Reading Data of Gyroscope and Accelerometer")

while True:
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0

    x = Ax * Ax
    y = Ay * Ay
    z = Az * Az
    acc = x + y + z
    a = acc ** 0.5

    if 0.9 < a < 1.2:
        string = 'static'
    elif 0.1 < a < 0.2:
        string = 'Decrease'
    elif a >= 1.3:
        string = 'impact'
    elif a < 0.9:
        string = 'fall'
    else:
        string = 'unknown'

    print(string)

    if string == 'fall':
        ser.write(str.encode("AT+CMGF=1\r"))
        print("Text mode enabled")
        time.sleep(3)
        ser.write(str.encode('AT+CMGS="9629204756"\r'))
        msg = "Fall Detected"
        print("sending message")
        time.sleep(3)
        ser.write(str.encode(msg + chr(26)))
        time.sleep(3)
        print("sent")
        ser.write(str.encode("ATD9629204756;\r"))
        print("Dialing")

        time.sleep(20)
        ser.write(str.encode("ATH\r"))
        print("Calling")
    time.sleep(0.002)
