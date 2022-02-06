import smbus		           #import SMBus module of I2C
from time import sleep          #import
import math
import time
import os                                # import os module
import glob
import RPi.GPIO as GPIO
import serial
import time,sys

SERIAL_PORT = "/dev/ttyS0"

ser = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout=5)


#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
#write to sample rate register
bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

#Write to power management register
bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

#Write to Configuration register
bus.write_byte_data(Device_Address, CONFIG, 0)

#Write to Gyro configuration register
bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

#Write to interrupt enable register
bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
#Accelero and Gyro value are 16-bit
high = bus.read_byte_data(Device_Address, addr)
low = bus.read_byte_data(Device_Address, addr+1)

#concatenate higher and lower value
value = ((high << 8) | low)

#to get signed value from mpu6050
if(value > 32768):
value = value - 65536
return value

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

while True:

#Read Accelerometer raw value
acc_x = read_raw_data(ACCEL_XOUT_H)
acc_y = read_raw_data(ACCEL_YOUT_H)
acc_z = read_raw_data(ACCEL_ZOUT_H)

#Read Gyroscope raw value
gyro_x = read_raw_data(GYRO_XOUT_H)
gyro_y = read_raw_data(GYRO_YOUT_H)
gyro_z = read_raw_data(GYRO_ZOUT_H)

#Full scale range +/- 250 degree/C as per sensitivity scale factor
Ax = acc_x/16384.0
Ay = acc_y/16384.0
Az = acc_z/16384.0

Gx = gyro_x/131.0
Gy = gyro_y/131.0
Gz = gyro_z/131.0

x=Ax*Ax
y=Ay*Ay
z=Az*Az
acc=x+y+z
a=acc**0.5
#angle=x/a
#theta=math.acos(angle)
count=0
if a<1.2 and a>0.9:
string='static'
elif a<0.2 and a>0.1:
string='Decrease'
elif a>=1.3:
string='impact'
elif a<0.9:
string='fall'
count=count+1
#print ("\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az,"\tA=%.2f g" %a)
print(string)
if(string=='fall'):
#print('fall')
ser.write(str.encode("AT+CMGF=1\r"))
print("Text mode enabled")
time.sleep(3)
ser.write(str.encode('AT+CMGS="9629204756"\r'))
msg="Fall Detected"
print("sending message")
time.sleep(3)
ser.write(str.encode(msg+chr(26)))
time.sleep(3)
print("sent")
ser.write(str.encode("ATD9629204756;\rs"))
print("Dialing")



time.sleep(20)
ser.write(str.encode("ATH\r"))
print("Calling")
sleep(0.002)
