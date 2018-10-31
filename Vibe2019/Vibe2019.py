#Some code gained from https://stackoverflow.com/questions/36172101/designate-specific-cpu-for-a-process-python-multiprocessing
#Using Pipes will block the code if the thread runs behind, halting further data from being gathered on the main thread.
import multiprocessing as mp
import DataServices
from multiprocessing import Process, Queue
import os
import TestStuff  as test    


if __name__ == '__main__':
    #test.StartCoreCalculations();
    #input()
    #test.StartMyCoreTest();

    test.QueueTest(1000000);