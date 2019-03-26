#Some code gained from https://stackoverflow.com/questions/36172101/designate-specific-cpu-for-a-process-python-multiprocessing
#Using Pipes will block the code if the thread runs behind, halting further data from being gathered on the main thread.
import multiprocessing as mp
import traceback
from multiprocessing import Process, Queue, Pool, Manager
import os
import TestStuff  as test    
import psutil
import time
import Gyro
import pickle
import serial
import datetime
from functools import partial
try:
    import gpiozero.spi_devices
except:
    print("Not on linux, can't access spi library")

# worker - the processor affinty this child works on
# childWorker - the processor affinity that will be handed off to another child process
# summaryQueue - a reference to the queue that the child will use.
def SPI_THREAD(worker: int, adcQueue: Queue, gyroDataQueue: Queue):
    try:
        p = psutil.Process()
        print(f"ADC Data Worker #{worker}: {p}, affinity {p.cpu_affinity()}", flush=True)
        time.sleep(1)
        p.cpu_affinity([worker])
        print(f"ADC Data Worker #{worker}: Set affinity to {worker}, affinity now {p.cpu_affinity()}", flush=True)
        
        ADC_Device : list = [] # List of devices that are really just representative of the pins on the ADC
        for pin in range(0,3): # Loop through each available pin
            ADC_Device.append( gpiozero.spi_devices.MCP3208(channel=pin, device=0) ) # channel is pin, device is CS.

        gyro = Gyro.ADIS16460(spiPort=0, spiCS=1) 

        #read default values
        print(f"MSC: {gyro.RegRead(gyro.MSC_CTRL)}")
        print(f"FLTR: {gyro.RegRead(gyro.FLTR_CTRL)}")
        print(f"DECR: {gyro.RegRead(gyro.DEC_RATE)}")

        while True:
            ADC_Values : list = []
            for device in ADC_Device:  # Print values per device  
                values : list = []
                values.append(device.value, device.voltage, device._channel) # values, and the channel id for matching
 
            burstArray = gyro.GetBurstData()           
            gyroDataQueue.put(burstArray)
            time.sleep(1)

            #print(f"Burst Data: {burstArray}") # Print array as string
            #print(f"Checksums match: {gyro.GetChecksum(burstArray)}") # Verify checksum value
            #gyro.PrintValues(burstArray) # Print gyro values after scaling
            
            if(adcQueue.qsize() < adcQueue.maxsize):
                adcQueue.put(ADC_Values)     

            if(gyroDataQueue.qsize() < gyroDataQueue.maxsize):
                gyroDataQueue.put(burstArray)
           
        print("ADC Data Worker Finished")
    except Exception as ex:
        print("Error in ADC_THREAD")
        traceback.print_exc()

def SEND_TELEM(list):
    #pass list to function
    #encode list as string
    #send data via serial
    ser = serial.Serial("/dev/serial0", 57600, timeout=3.0)
    telemString = Str(list).encode()
    ser.write(telemString)

def TELEMETRY(worker: int, toSendQueue: Queue):
    #Check each queue to see if they have a size of 4000 or greater
    #If so, send the data via telemetry at a maximum of 4000 data points
    #priority: ADC first, Gyro second
    telemList : list = [4000]
    telemString = ""
    cTime : datetime = datetime.datetime.now()
    crashNum : int = 0
    delta : datetime.timedelta
    while True:
        try:
            if telemList.len == 4000:
                try:
                    SEND_TELEM(telemList)
                    telemList = [4000]
                except:
                    pass
            else:
                telemList.append(adcQueue.get())
        except:
            #store the number of crashes            
            crashNum += 1
        finally:
            #check current time against a time variable
            #every five minutes replace the value of the time variable
            #send the number of crashes every 5 minutes
            delta = datetime.datetime.now() - cTime
            if delta.total_seconds() >= 300:
                print("Telemetry Thread has crashed ", crashNum, " times since thread began.")
                cTime = datetime.datetime.now()

                #make a log file and include time
    
    pass


if __name__ == '__main__':
    m = Manager()
    adcDataQueue = m.Queue(maxsize=6000)
    gyroDataQueue = m.Queue(maxsize=6000)

    process_SPI =  Process(target=SPI_THREAD, args=(1, adcDataQueue, gyroDataQueue))
    process_Telemetry =  Process(target=TELEMETRY, args=(2, adcDataQueue, gyroDataQueue))

    try:  
        process_SPI.start()
        process_Telemetry.start()
        
    except Exception as ex:
        print("Error in Main")
        print(ex)

    process_SPI.join()
    process_Telemetry.join()

