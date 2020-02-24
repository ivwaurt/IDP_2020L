import serial
import time


data = serial.Serial('com5',9600)
time.sleep(1)


t = 1
while t<10000:
    time.sleep(1)
    output = data.readline()
    print(data,output)
    t = t+1
