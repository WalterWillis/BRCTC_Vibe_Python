#Some code gained from https://stackoverflow.com/questions/36172101/designate-specific-cpu-for-a-process-python-multiprocessing
#Using Pipes will block the code if the thread runs behind, halting further data from being gathered on the main thread.
import multiprocessing as mp
import DataServices
from multiprocessing import Process, Queue, Pool, Manager
import os
import TestStuff  as test    
import psutil
import time
from functools import partial
try:
    import gpiozero.spi_devices
except:
    print("Not on linux, can't access spi library")

# worker - the processor affinty this child works on
# childWorker - the processor affinity that will be handed off to another child process
# summaryQueue - a reference to the queue that the child will use.
def ADC_THREAD(worker: int, adcQueue: Queue):
    try:
        try:
            ADC1 = gpiozero.spi_devices.MCP3208(0) # should be channel 0. Will need a variable, or array for all channels used.
            print("ADC channel is channel {ADC1.channel}")

        except:
            print("Error creating ADC object")

        p = psutil.Process()
        print(f"ADC Data Worker #{worker}: {p}, affinity {p.cpu_affinity()}", flush=True)
        time.sleep(1)
        p.cpu_affinity([worker])
        print(f"ADC Data Worker #{worker}: Set affinity to {worker}, affinity now {p.cpu_affinity()}", flush=True)

        adcQueue.put("ADC DATA!")     
           
        print("ADC Data Worker Finished")
    except Exception as ex:
        print("Error in ADC_THREAD");
        print(ex);

def GYRO_THREAD(worker: int, gyroQueue: Queue):
    try:
        p = psutil.Process()
        print(f"Gyro Data Worker #{worker}: {p}, affinity {p.cpu_affinity()}", flush=True)
        time.sleep(1)
        p.cpu_affinity([worker])
        print(f"Gyro Data Worker #{worker}: Set affinity to {worker}, affinity now {p.cpu_affinity()}", flush=True)

        gyroQueue.put("GYRO DATA")   
            
        print("Gyro Data Worker Finished")
    except Exception as ex:
        print("Error in GYRO_THREAD");
        print(ex);


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
                break;
            time.sleep(.5)

        print("ADC Handler Finished")
    except Exception as ex:
        print("Error in ADC_Handler");
        print(ex);

def GYRO_HANDLER(worker: int, gyroQueue: Queue, gyroSummaryQueue: Queue):
    try:
        p = psutil.Process()
        print(f"Gyro Data Handler #{worker}: {p}, affinity {p.cpu_affinity()}", flush=True)
        time.sleep(1)
        p.cpu_affinity([worker])
        print(f"Gyro Data Handler #{worker}: Set affinity to {worker}, affinity now {p.cpu_affinity()}", flush=True)

        while(True):
            if(gyroQueue.empty() == False):
                message = gyroQueue.get()
                gyroSummaryQueue.put(message)
                break;
            time.sleep(.5)

        print("Gyro Handler Finished")
    except Exception as ex:
        print("Error in GYRO_Handler");
        print(ex);

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
    #test.StartCoreCalculations();
    #input()
    #test.StartMyCoreTest();

    #The main thread should be the data handler for the sake of efficiency and programmatic simplicity

    #main()
    m = Manager()
    adcDataQueue = m.Queue();
    gyroDataQueue = m.Queue();
    adcSummaryQueue = m.Queue();
    gyroSummaryQueue = m.Queue();
    summaryQueueSender = m.Queue();

    p1 = Process(target=GYRO_HANDLER, args=(1, gyroDataQueue, gyroSummaryQueue))
    p2 = Process(target=ADC_HANDLER, args=(2, adcDataQueue, adcSummaryQueue))
    p3 = Process(target=GYRO_THREAD, args=(3, gyroDataQueue))
    p4 = Process(target=ADC_THREAD, args=(4, adcDataQueue))

    try:       
        p1.start()
        p2.start()
        p3.start()
        p4.start()
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
    p3.join()
    p4.join()
    print("Done")

