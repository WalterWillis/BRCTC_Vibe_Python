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
        
        #ADC_Device : list = []
        # for pin in range(0,7):
        #     ADC_Device.append( gpiozero.spi_devices.MCP3208(channel=pin, device=0) ) # channel is pin, device is CS.

        # for i in range(0,200):   
        #     for device in ADC_Device:                             
        #         print(f"ADC channel is channel {device.channel}", flush=True)
        #         print(f"ADC voltage is {device.voltage} volts", flush=True)
        #         print(f"ADC value is {device.value}", flush=True)
        #         time.sleep(.3)

        gyro = Gyro.ADIS16460(0,1) 

        #read default values
        print(f"MSC : {gyro.RegRead(gyro.MSC_CTRL)}")
        print(f"FLTR: {gyro.RegRead(gyro.FLTR_CTRL)}")
        print(f"DECR: {gyro.RegRead(gyro.DEC_RATE)}")

        while True:
            burstArray = gyro.GetBurstData()           
            gyroDataQueue.put(burstArray)
            time.sleep(1)
            

        adcQueue.put("ADC DATA!")     
           
        print("ADC Data Worker Finished")
    except Exception as ex:
        print("Error in ADC_THREAD")
        traceback.print_exc()


def ADC_HANDLER(worker: int, adcQueue: Queue, adcSummaryQueue: Queue):
    try:
        p = psutil.Process()
        print(f"ADC Data Handler #{worker}: {p}, affinity {p.cpu_affinity()}", flush=True)
        time.sleep(1)
        p.cpu_affinity([worker])
        print(f"ADC Data Handler #{worker}: Set affinity to {worker}, affinity now {p.cpu_affinity()}", flush=True)

        while(True):
            if(adcQueue.empty() == False):
                message = adcQueue.get()
                adcSummaryQueue.put(message)
                break
            time.sleep(.5)

        print("ADC Handler Finished")
    except Exception as ex:
        print("Error in ADC_Handler")
        print(ex)
        print(traceback)

def GYRO_HANDLER(worker: int, gyroDataQueue: Queue, gyroSummaryQueue: Queue):
    try:
        p = psutil.Process()
        print(f"Gyro Data Handler #{worker}: {p}, affinity {p.cpu_affinity()}", flush=True)
        time.sleep(1)
        p.cpu_affinity([worker])
        print(f"Gyro Data Handler #{worker}: Set affinity to {worker}, affinity now {p.cpu_affinity()}", flush=True)

        while(True):
            if(gyroQueue.empty() == False):
                message = gyroDataQueue.get()
                print(f"Burst Data: {message}", flush=True)
                print(f"Checksums match: {Gyro.GetChecksum(message)}", flush=True)
                Gyro.PrintStuff(message)
                gyroSummaryQueue.put(message)
                break
            time.sleep(.5)

        print("Gyro Handler Finished")
    except Exception as ex:
        print("Error in GYRO_Handler")
        print(ex)

def TELEMETRY(worker: int, toSendQueue: Queue):
    pass

#--------Test Stuff-------
def func(a, b):
    print("hi")
    return a + b

def main():
    a_args = [1,2,3]
    second_arg = 1
    with Pool() as pool:
        L = pool.starmap(func, [(1, 1), (2, 1), (3, 1)])
        print(L)
#--------------------------


if __name__ == '__main__':
    #test.StartCoreCalculations()
    #input()
    #test.StartMyCoreTest()

    #The main thread should be the data handler for the sake of efficiency and programmatic simplicity

    #main()
    m = Manager()
    adcDataQueue = m.Queue()
    gyroDataQueue = m.Queue()
    adcSummaryQueue = m.Queue()
    gyroSummaryQueue = m.Queue()
    summaryQueueSender = m.Queue()

    process_SPI =  Process(target=SPI_THREAD, args=(1, adcDataQueue, gyroDataQueue))

    p1 = Process(target=GYRO_HANDLER, args=(2, gyroDataQueue, gyroSummaryQueue))
    p2 = Process(target=ADC_HANDLER, args=(3, adcDataQueue, adcSummaryQueue))

    try:  
        process_SPI.start()
        p1.start()
        p2.start()
        
    except Exception as ex:
        print("Error in Main")
        print(ex)

    #Breaks the loop
    gyroReady = False
    adcReady = False

    while(not gyroReady and not adcReady):

        gyroReady = gyroSummaryQueue.empty()
        adcReady = adcSummaryQueue.empty()

        if(gyroReady):
            print("Message from Gyro Summary Queue")
            print(gyroSummaryQueue.get())

        if(adcReady):
            print("Message from ADC Summary Queue")
            print(adcSummaryQueue.get())       

        time.sleep(.5)

    p1.join()
    p2.join()
    process_SPI.join()
    print("Done")

