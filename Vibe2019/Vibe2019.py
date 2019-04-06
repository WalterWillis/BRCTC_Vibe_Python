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
    
def fill(data, q: Queue): #inspired by the drain function
    try:
        q.put(data,block=True,timeout=0.1)
    except:
        print("Error in fill")

def drain(q: Queue): #https://stackoverflow.com/questions/21157739/how-to-iterate-through-a-python-queue-queue-with-a-for-loop-instead-of-a-while-l  
    try:
        return q.get(block=True,timeout=1)  
    except:
        print("Error in drain")



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

        queueList = []
        listLength = 4000
        while True:
            try:
                ADC_Values : list = []
                for device in ADC_Device:  # Print values per device  
                    values : list = []
                    values.append(device.value, device.voltage, device._channel, datetime.datetime.now().time()) # values, and the channel id for matching
                    ADC_Values += values
 
                queueList.append(Loop_Values + gyro.GetBurstData()) #[0, 1, 2, 3, 4] + [5, 6, 7, 8, 9] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
                
                if(len(queueList) >= listLength):
                    fill(queueList, dataQueue)
                    fill(queueList, telemQueue)

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
    #priority: ADC first, Gyro second
    telemString = ""
    cTime : datetime = datetime.datetime.now()
    crashNum : int = 0
    delta : datetime.timedelta
    ser = serial.Serial("/dev/serial0", 57600, timeout=3.0, write_timeout=3.0)
    while True:
        try:
            data = drain(telemQueue)
            if(data != None):
                ser.write(data.encode())            
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
  
def DATA_HANDLING(worker: int, dataQueue: Queue):
    p = psutil.Process()   
    time.sleep(1)
    p.cpu_affinity([worker])

    itemList : list = []
    writeCounter : int = 0
    fileHeader = "adcValue0 , adcVoltage0 , adcChannel0 , date , adcValue1 , adcVoltage1 , adcChannel1 , date , adcValue2 , adcVoltage2 , adcChannel2 , date , gyro_DIAG_STAT, gyro_XGYRO , gyro_YGYRO , gyro_ZGYRO , gyro_XACCEL , gyro_YACCEL , gyro_ZACCEL , gyro_TEMP_OUT , gyro_SMPL_CNTR , gyro_CHECKSUM , date"
    
    #Directory should be the date of the test
    #Add a global variable to a hard-coded directory later such as /home/pi/Desktop
    directoryName = datetime.datetime.now().strftime("%c")
    #create dynamically named file
    fileCounter = 0
    file = str(fileCounter) + ".txt"

    with open(file, 'a') as f:
        f.write(fileHeader)
        f.write("\n")

    while True:
        #get items from dataQueue
        data = drain(dataQueue)

        if data != None:
            with open(file, 'a') as f:
                f.write("START OF DATA\n")
                json.dump(fileList, f,  indent=4)
                f.write("END OF DATA\n")
            fileList : list = []
            writeCounter += 1

            #Create a new file after a million datapoints
            if writeCounter >= 250: #4000 x 250 = 1000000
                fileCounter += 1
                file = str(fileCounter) + ".txt"

       

if __name__ == '__main__':
    m = Manager()
    #The queues will hold identical data
    dataQueue : Queue = m.Queue(maxsize = 60)
    telemQueue: Queue = m.Queue(maxsize = 60)

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

   

