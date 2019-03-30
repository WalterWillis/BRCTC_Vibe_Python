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
import datetime
from functools import partial
try:
    import serial
    import gpiozero.spi_devices
except:
    print("Not on linux, can't access spi and serial library")
    
    #lower case queue needs to be tested
def fill(data, q: Queue): #inspired by the drain function
  while True:
    try:
      q.put(data,block=true,timeout=0.1)
    except queue.Full:  # on python 2 use Queue.Empty
      break

def drain(q: Queue): #https://stackoverflow.com/questions/21157739/how-to-iterate-through-a-python-queue-queue-with-a-for-loop-instead-of-a-while-l
  while True:
    try:
      yield q.get_nowait()
    except queue.Empty:  # on python 2 use Queue.Empty
      break


# worker - the processor affinty this child works on
# childWorker - the processor affinity that will be handed off to another child process
# summaryQueue - a reference to the queue that the child will use.
def SPI_THREAD(worker: int, dataQueue: Queue, telemQueue: Queue):
    try:
        p = psutil.Process()
        #print(f"ADC Data Worker #{worker}: {p}, affinity {p.cpu_affinity()}", flush=True)
        #time.sleep(1)
        p.cpu_affinity([worker])
        #print(f"ADC Data Worker #{worker}: Set affinity to {worker}, affinity now {p.cpu_affinity()}", flush=True)
        
        ADC_Device : list = [] # List of devices that are really just representative of the pins on the ADC
        for pin in range(0,3): # Loop through each available pin
            ADC_Device.append( gpiozero.spi_devices.MCP3208(channel=pin, device=0) ) # channel is pin, device is CS.

        gyro = Gyro.ADIS16460(spiPort=0, spiCS=1) 

        #read default values
        print( f"MSC: {gyro.RegRead(gyro.MSC_CTRL)}")
        print(f"FLTR: {gyro.RegRead(gyro.FLTR_CTRL)}")
        print(f"DECR: {gyro.RegRead(gyro.DEC_RATE)}")

        while True:
            try:
                Loop_Values : list = []
                for device in ADC_Device:  # Print values per device  
                    values : list = []
                    values.append(device.value, device.voltage, device._channel, datetime.datetime.now().time()) # values, and the channel id for matching
                    Loop_Values += values
 
                Loop_Values + gyro.GetBurstData() #[0, 1, 2, 3, 4] + [5, 6, 7, 8, 9] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
                fill(values, dataQueue)
                fill(values, telemQueue)

                #print(f"Burst Data: {burstArray}") # Print array as string
                #print(f"Checksums match: {gyro.GetChecksum(burstArray)}") # Verify checksum value
                #gyro.PrintValues(burstArray) # Print gyro values after scaling
            except Exception as ex:
                pass
           
        print("ADC Data Worker Finished")
    except Exception as ex:
        print("Error in ADC_THREAD")
        traceback.print_exc()
        raise ex #Throw the error to restart the program.


def TELEMETRY(worker: int, telemQueue: Queue):
    p = psutil.Process()
    p.cpu_affinity([worker])

    #Check each queue to see if they have a size of 4000 or greater
    #If so, send the data via telemetry at a maximum of 4000 data points
    listLength : int = 4000
    #priority: ADC first, Gyro second
    telemList : list = [listLength]
    telemString = ""
    cTime : datetime = datetime.datetime.now()
    crashNum : int = 0
    delta : datetime.timedelta
    ser = serial.Serial("/dev/serial0", 57600, timeout=3.0, write_timeout=3.0)
    while True:
        try:
            if telemList.len == listLength:
                try:
                    ser.write(telemList.encode())
                    telemList = [listLength]
                except:
                    pass
            else:
                for data in drain(telemQueue):
                    telemList.append(data)
                    if telemList.len == listLength: #Don't go over the list length
                        break;
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
    


if __name__ == '__main__':
    m = Manager()
    #The queues will hold identical data
    dataQueue : Queue = m.Queue(maxsize = 6000)
    telemQueue: Queue = m.Queue(maxsize = 6000)

    process_SPI =  Process(target=SPI_THREAD, args=(1, dataQueue, telemQueue))
    process_DataHandler =  Process(target=TELEMETRY, args=(2, dataQueue))
    process_Telemetry =  Process(target=TELEMETRY, args=(3, telemQueue))

    while True:
        try:  
            process_SPI.start()
            process_Telemetry.start()
            process_DataHandler.start()

            process_SPI.join()
            process_Telemetry.join()
            process_DataHandler.join()
        except Exception as ex:
            print("Error in Main")
            print(ex)
            try:
                if(process_SPI.is_alive()):
                    process_SPI.terminate()
                if(process_Telemetry.is_alive()):
                    process_Telemetry.terminate()
            except:
                print("Error terminating processes")
                #A catastrophic failure is likely to have occurred if we are here. 
                #We can assume that all resources will be removed when returning through the loop.

   

