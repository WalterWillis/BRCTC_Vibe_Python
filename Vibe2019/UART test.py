import serial
from time import sleep

ser = serial.Serial("/dev/serial0", 57600, timeout=3.0)
num = 0
while True:
    print(num)
    message = f"{num}\n"
    num += 1
    message += f"{num}\n"
    num += 1
    message += f"{num}\n"
    num += 1
    message += f"{num}\n"
    ser.write(message.encode())
    #ser.flush()
    num += 1
    #sleep(1)
    
    
    