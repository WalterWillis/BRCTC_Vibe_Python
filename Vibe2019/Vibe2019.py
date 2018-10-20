#Some code gained from https://stackoverflow.com/questions/36172101/designate-specific-cpu-for-a-process-python-multiprocessing

import multiprocessing as mp
from multiprocessing import Process, Pipe
import os


class Thread:
    def __init__(self, name):
        self.parentConnection, self.childConnection =  Pipe()
        self.name = name
        self.Process = Process(target=childLogic, args=(self.name, self.childConnection))

        

def child(worker: int) -> None:
    import psutil
    import time

    p = psutil.Process()
    print(f"Child #{worker}: {p}, affinity {p.cpu_affinity()}", flush=True)
    time.sleep(1)
    p.cpu_affinity([worker])
    print(f"Child #{worker}: Set my affinity to {worker}, affinity now {p.cpu_affinity()}", flush=True)

    time.sleep(1 + 3 * worker)
    print(f"Child #{worker}: Starting CPU intensive task now for 4 seconds on {p.cpu_affinity()}...", flush=True)
    t_end = time.perf_counter() + 100
    while time.perf_counter() < t_end:
        pass
    print(f"Child #{worker}: Finished CPU intensive task on {p.cpu_affinity()}", flush=True)


def main() -> None:
    with mp.Pool() as pool:
        # noinspection PyProtectedMember
        workers: int = pool._processes
        print(f"Running pool with {workers} workers")

        for i in range(workers):
            pool.apply_async(child, (i,))

        # Wait for children to finnish
        pool.close()
        pool.join()

    pass




def info(title):
    print( title)
    print('module name:', __name__)
    if hasattr(os, 'getppid'):  # only available on Unix
        print('parent process: ')
        print(os.getppid())
    print('process id:')
    print(os.getpid())

def childLogic(name, conn):
    info('Calling chilLogic')
    print('hello ' + name)
    PipeTest(conn,"Hello from " + name)
    while True:
        print(name, 'cant stop, wont stop')

def PipeTest(conn, data):
    conn.send(data)
    conn.close()

if __name__ == '__main__':
    info('main process')
    print(os.environ['NUMBER_OF_PROCESSORS'])

    main()
    input()

    ThreadList = [Thread('Thread' + str(count)) for count in range(0,12)]
    for thread in ThreadList:
        thread.Process.start()

    #parentConnection, childConnection =  Pipe()
    #parentConnection2, childConnection2 =  Pipe()
    #p = Process(target=childLogic, args=('Bob', childConnection))
    #v = Process(target=childLogic, args=('Julie', childConnection2))
    #input()
    #p.start()
    #v.start()
    #print(parentConnection.recv())
    #PipeTest(parentConnection, 'Hello Bob')
    #print(parentConnection2.recv())
    #PipeTest(parentConnection2, 'Hello Julie')
    #p.join()
    #v.join()
