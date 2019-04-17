#Some code gained from https://stackoverflow.com/questions/36172101/designate-specific-cpu-for-a-process-python-multiprocessing

#imports
#region
import multiprocessing
from multiprocessing import Process, Queue, Pool, Manager
import queue
import os
#import TestStuff  as test    
import psutil
import time
import logging

import datetime
from functools import partial
try:
    import Gyro
    import serial
    import gpiozero.spi_devices
    import SDL_DS3231
except:
    #logger.exception("Error importing linux libraries!")
    print("Not on linux")
#endregion

#globals
#region
#global variable; instantiates real time clock
RTC = SDL_DS3231.SDL_DS3231(1, 0x68)
try:
    os.system('sudo rmmod rtc_ds1307')
except:
    pass

#Multithreading lock
lock = multiprocessing.Lock()

#Naming main directory of things
MainDir = None

try:
    MainDir = "/home/pi/Desktop/Vibe_" + GetTime().strftime("%c")+"/"
except:
    #In windows, we will only except things to be written in the current directory
    MainDir = "./"

#creating directory if does not exist
if not os.path.exists(MainDir):
    os.makedirs(MainDir)

#logger info
logFileName = MainDir + "Log.txt"
logging.basicConfig(filename=logFileName, level=logging.DEBUG, format='%(asctime)s %(levelname)s %(name)s %(message)s')
logger=logging.getLogger(__name__)

#endregion

#shared functions
#region

#shares a global variable
def GetTime():
    counter = 0;

    date : datetime.datetime = None
    try:          
        lock.acquire()  
        date = RTC.read_datetime()
        time.sleep(2)
        lock.release()
    except:            
        counter += 1

    #if the lock could never acquire, or I2C is down, just get a datetime
    if date == None:
        print("Error")
        date = datetime.datetime.now()
        logger.info("Could not access RTC, getting system datetime")
    
    return date

    
def fill(data, q: Queue): #inspired by the drain function
    try:
        q.put(data,block=True,timeout=0.1)
    except:
        logger.exception("Error in fill")

def drain(q: Queue): #https://stackoverflow.com/questions/21157739/how-to-iterate-through-a-python-queue-queue-with-a-for-loop-instead-of-a-while-l  
    try:
        return q.get(block=True,timeout=1)  
    except:
        logger.exception("Error in drain")

#endregion

#threads
#region

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
                    values.append(device.value, device.voltage, device._channel, GetTime()) # values, and the channel id for matching
                    ADC_Values += values
 
                queueList.append(Loop_Values + gyro.GetBurstData()+[GetTime()]) #[0, 1, 2, 3, 4] + [5, 6, 7, 8, 9] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
                
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
        logger.exception("Error in ADC_THREAD")
        raise ex #Throw the error to restart the program.


def TELEMETRY(worker: int, telemQueue: Queue):
    p = psutil.Process()
    p.cpu_affinity([worker])

    #Check each queue to see if they have a size of 4000 or greater
    #If so, send the data via telemetry at a maximum of 4000 data points
    #priority: ADC first, Gyro second
    telemString = ""
    cTime : datetime = GetTime()
    crashNum : int = 0
    delta : datetime.timedelta
    ser = serial.Serial("/dev/serial0", 57600, timeout=3.0, write_timeout=3.0)
    while True:
        try:
            data = drain(telemQueue)
            if(data != None):
                ser.write(data.encode())            
        except:
            logger.exception("Telemetry Thread has crashed")
            #store the number of crashes            
            crashNum += 1
            delta = GetTime() - cTime
            if delta.total_seconds() >= 300:
                logger.info("Telemetry Thread has crashed ", crashNum, " times since the thread began.")
                cTime = GetTime()
            #we took out the code below because the logger (above) can handle all of it in one foul swoop
            #finally:
            #check current time against a time variable
            #every five minutes replace the value of the time variable
            #send the number of crashes every 5 minutes
            #delta = GetTime() - cTime
            #if delta.total_seconds() >= 300:
                #print("Telemetry Thread has crashed ", crashNum, " times since thread began.")
                #cTime = GetTime()

                #make a log file and include time
  
def DATA_HANDLING(worker: int, dataQueue: Queue):
    p = psutil.Process()   
    time.sleep(1)
    p.cpu_affinity([worker])

    itemList : list = []
    writeCounter : int = 0
    fileHeader = "adcValue0 , adcVoltage0 , adcChannel0 , date , adcValue1 , adcVoltage1 , adcChannel1 , date , adcValue2 , adcVoltage2 , adcChannel2 , date , gyro_DIAG_STAT, gyro_XGYRO , gyro_YGYRO , gyro_ZGYRO , gyro_XACCEL , gyro_YACCEL , gyro_ZACCEL , gyro_TEMP_OUT , gyro_SMPL_CNTR , gyro_CHECKSUM , date"
    
    directoryName = MainDir+"Data/"
    if not os.path.exists(directoryName):
        os.makedirs(directoryName)
    #create dynamically named file
    fileCounter = 0
    file = directoryName+str(fileCounter) + ".txt"

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
                file = directoryName+str(fileCounter) + ".txt"

#endregion
 
#test threads
#region 
def placein(id: int, dataQueue : Queue, telemQueue: Queue):
    for i in range(0, 100):
        fill("Hi", dataQueue)
        f = drain(telemQueue)
        if f != None and f == "ready":
            print(f)
            fill("Hi", dataQueue)

def takeout(id: int, dataQueue: Queue, telemQueue: Queue):
    counter = 0
    while counter < 100:
        s = str(drain(dataQueue))
        if(s != None and len(s) > 0):
            print(s)
            fill("ready", telemQueue)
            counter +=1
        else:
            print("variable s is empty")
#endregion

#Main Thread
#region 
#differentiates processes. This is the main process
if __name__ == '__main__':
    try:
        multiprocessing.set_start_method('spawn')
    except:
        logger.exception("Setting start method has crashed")
    m = Manager()
    #The queues will hold identical data
    dataQueue : Queue = m.Queue(maxsize = 60)
    telemQueue: Queue = m.Queue(maxsize = 60)       

    while True:
        try:  

            #Must define inside of the loop in order to restart after a crash
            process1 = Process(target=placein, args=(2, dataQueue, telemQueue))
            process2 = Process(target=takeout, args=(3, dataQueue, telemQueue))
            process1.start()
            process2.start()

            process1.join()
            process2.join()

            process1.terminate()
            process2.terminate()

            #process_SPI =  Process(target=SPI_THREAD, args=(1, dataQueue, telemQueue))
            #process_DataHandler =  Process(target=TELEMETRY, args=(2, dataQueue))
            #process_Telemetry =  Process(target=TELEMETRY, args=(3, telemQueue))
            
            #process_SPI.start()
            #process_Telemetry.start()
            #process_DataHandler.start()

            #process_SPI.join()
            #process_Telemetry.join()
            #process_DataHandler.join()
            

            print("Finished!")


        except:
            logger.exception("Error in Main")
            #traceback.print_exc()
            input()
            try:
                if(process_SPI.is_alive()):
                    process_SPI.terminate()
                if(process_Telemetry.is_alive()):
                    process_Telemetry.terminate()
            except:
                logger.exception("Error terminating processes")
                #A catastrophic failure is likely to have occurred if we are here. 
                #We can assume that all resources will be removed when returning through the loop.

        time.sleep(20)
    

#endregion