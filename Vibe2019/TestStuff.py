import multiprocessing as mp
from multiprocessing import Process, Pipe, Queue
import multiprocessing.queues as mpq
import os

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

#My first test
def MyFisrtTest():
    parentConnection, childConnection =  Pipe()
    parentConnection2, childConnection2 =  Pipe()
    p = Process(target=childLogic, args=('Bob', childConnection))
    v = Process(target=childLogic, args=('Julie', childConnection2))
    input()
    p.start()
    v.start()
    print(parentConnection.recv())
    PipeTest(parentConnection, 'Hello Bob')
    print(parentConnection2.recv())
    PipeTest(parentConnection2, 'Hello Julie')
    p.join()
    v.join()

#Found from the internet
def StartCoreCalculations():
    info('main process')
    print(os.environ['NUMBER_OF_PROCESSORS'])

    main()

#My second test
def StartMyCoreTest():
    ThreadList = [Thread('Thread' + str(count)) for count in range(0,12)]
    for thread in ThreadList:
        thread.Process.start()


def QueueTest_Child(_q, c_q):
    import time
    num = 0
    for i in range(0,60): # 60 iterations,with half a second delay ~ 30 seconds
        message = ""
        block = True
        while block:
            try:
                message = _q.get() # block
                block = False
            except: pass
        print(message)
        time.sleep(.5)
        c_q.put_nowait(" Child Recieved at cycle: {0}".format(i))
    c_q.put_nowait("Exit")

def QueueTest(iterations):
    q = Queue()
    c_q = Queue()
    p = Process(target=QueueTest_Child, args=(q,c_q,))
    p.start()
    count = 0
    for count in range(0, iterations):
        if q.empty()and c_q.empty():
            q.put_nowait("Parent sent at cycle: {0}".format(count))
        if c_q.empty() != True:
            message = c_q.get()
            if message == "Exit":
                break
            try:print(message)
            except: pass
    print("Done!")
    p.join()




class Thread:
    def __init__(self, name):
        self.parentConnection, self.childConnection =  Pipe()
        self.name = name
        self.Process = Process(target=childLogic, args=(self.name, self.childConnection))

  
class MyQueue(mpq.Queue):
    '''
    A custom queue subclass that provides a :meth:`clear` method.
    '''
  
    def __init__(self,*args,**kwargs):
        ctx = mp.get_context()
        super(MyQueue, self).__init__(*args, **kwargs, ctx=ctx)

    def clear(self):
        '''
        Clears all items from the queue.
        '''

        unfinished = self.unfinished_tasks - len(self.queue)
        if unfinished <= 0:
            if unfinished < 0:
                raise ValueError('task_done() called too many times')
            self.all_tasks_done.notify_all()
        self.unfinished_tasks = unfinished
        self.queue.clear()
        self.not_full.notify_all()