#!/usr/bin/env python

from hardware import *
import log
TERMINATED = "TERMINATED"
RUNNING = "RUNNING"
READY = "READY"
WAITING = "WAITING"
NEW = "NEW"
TICKIO = 3


## emulates a compiled program
class Program():

    def __init__(self, name, instructions):
        self._name = name
        self._instructions = self.expand(instructions)

    @property
    def name(self):
        return self._name

    @property
    def instructions(self):
        return self._instructions

    def addInstr(self, instruction):
        self._instructions.enqueue(instruction)

    def expand(self, instructions):
        expanded = []
        for i in instructions:
            if isinstance(i, list):
                ## is a list of instructions
                expanded.extend(i)
            else:
                ## a single instr (a String)
                expanded.append(i)

        ## now test if last instruction is EXIT
        ## if not... add an EXIT as final instruction
        last = expanded[-1]
        if not ASM.isEXIT(last):
            expanded.append(INSTRUCTION_EXIT)

        return expanded

    def __repr__(self):
        return "Program({name}, {instructions})".format(name=self._name, instructions=self._instructions)


## emulates an Input/Output device controller (driver)
class IoDeviceController():

    def __init__(self, device):
        self._device = device
        self._waiting_queue = Waiting_Queue()
        self._currentPCB = None

    def runOperation(self, pcb, instruction):
        log.logger.info("agregando pcb {} instr {}".format(pcb,instruction))
        # enqueue: adds the element at the end of the queue
        pcb.state = WAITING
        if HARDWARE.ioDevice.is_idle:
            self._device.execute(instruction)
            self._currentPCB = pcb
        else:
            self._waiting_queue.enqueue(pcb, instruction)
        # try to send the instruction to hardware's device (if is idle)
        

    def getFinishedPCB(self):
        finishedPCB = self._currentPCB
        self._currentPCB = None
        return finishedPCB

    def loadFromWaitingQueueIfItApplies(self):
        if not self._waiting_queue.isEmpty() and self._device.is_idle:
            
            pair = self._waiting_queue.dequeue()
            pcb = pair['pcb']
            instruction = pair['instruction']
            self._currentPCB = pcb
            self._device.execute(instruction)


    def __repr__(self):
        return "IoDeviceController for {deviceID} running: {currentPCB} waiting: {waiting_queue}".format(deviceID=self._device.deviceId, currentPCB=self._currentPCB, waiting_queue=self._waiting_queue)

## emulates the  Interruptions Handlers
class AbstractInterruptionHandler():
    def __init__(self, kernel):
        self._kernel = kernel

    @property
    def kernel(self):
        return self._kernel

    def execute(self, irq):
        log.logger.error("-- EXECUTE MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))


class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        log.logger.info(" Program Finished ")
        log.logger.info("readyQueue {}".format(self.kernel.scheduler.readyQ))
        procesoRunning = self.kernel.pcbTable.pcbRunning()
        log.logger.info("proceso corriendo {}".format(procesoRunning))
        if (procesoRunning != None):
            procesoRunning.state = TERMINATED
            self.kernel.dispatcher.save(procesoRunning)
        if(not self.kernel.scheduler.readyQ.isEmpty()):
            next_pcb = self.kernel.scheduler.nextPCB()
            self.kernel.dispatcher.load(next_pcb)
        elif(self.kernel.pcbTable.todosPCBsTerminados()):##todos los procesos de pcbTable estan terminados
            HARDWARE.switchOff()
            log.logger.info("Gantt {}".format(self.kernel.gantt))

class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        operation = irq.parameters
        pcbrunning = self.kernel.pcbTable.pcbRunning()
        log.logger.info("{}".format(pcbrunning))
        
        self.kernel.dispatcher.save(pcbrunning)
        self.kernel.ioDeviceController.runOperation(pcbrunning, operation)
        log.logger.info(self.kernel.ioDeviceController)
        
        if(not self.kernel.scheduler.readyQ.isEmpty()):
            next_pcb = self.kernel.scheduler.nextPCB()
            self.kernel.dispatcher.load(next_pcb)

class NewInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        program = irq.parameters["program"]
        prioridad = irq.parameters["priority"]
        baseDir = self.kernel.loader.load(program)
        pcb = PCB()
        pcb.pcb(program, baseDir, prioridad)
        self.kernel.pcbTable.add(pcb)
        self.kernel.scheduler.add(pcb)
        
        
        log.logger.info("\n Executing program: {name}".format(name=program.name))
        log.logger.info("\n Diccionario: {pcbTable}".format(pcbTable=pcb))
        log.logger.info(HARDWARE)


class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self.kernel.ioDeviceController.getFinishedPCB()
        
        self.kernel.scheduler.add(pcb)
            
        self.kernel.ioDeviceController.loadFromWaitingQueueIfItApplies()
        log.logger.info(self.kernel.ioDeviceController)

class TimeOutHandler(AbstractInterruptionHandler):
   
    def execute(self, irq):
        
        if not self.kernel.scheduler.isEmpty() and self.kernel.pcbTable.pcbRunning() != None:
            pcbRunning = self.kernel.pcbTable.pcbRunning()
            self.kernel.scheduler.expropiarPCB(pcbRunning)
        HARDWARE.timer.reset()
        


# emulates the core of an Operative System
class Kernel():

    def __init__(self):
        self.loader = Loader()
        self.pcbTable = PCBTable()
        
        #self.scheduler = FCFS(self)
        #self.scheduler = PrioridadNoExpropiativa(self)
        #self.scheduler = PrioridadExpropiativa(self)
        self.scheduler = RoundRobin(self)
        HARDWARE.timer.quantum = 3

        self.dispatcher = Dispatcher()
        self.gantt = Gantt(self)
        HARDWARE.clock.addSubscriber(self.gantt)
        ## setup interruption handlers
        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        newHandler = NewInterruptionHandler(self)
        
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)
        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)

        timeHandler = TimeOutHandler(self)
        HARDWARE.interruptVector.register(TIMEOUT_INTERRUPTION_TYPE, timeHandler)
        

    @property
    def ioDeviceController(self):
        return self._ioDeviceController

    ## emulates a "system call" for programs execution
    def run(self, program, prioridad = 1):
        pair = {"program": program, "priority": prioridad}
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, pair)
        HARDWARE.interruptVector.handle(newIRQ)
        

    def __repr__(self):
        return "Kernel "

    def executeBatch(self, batch):
        program = batch
        for i in program:
            self.run(i)


class Loader():

    def __init__(self):
        self.indicePrograma = 0

    def load(self, program):
        progSize = len(program.instructions)
        log.logger.info("\n Program Size antes de cargar: {size}".format(size=progSize))
        log.logger.info("\n Indice de programa antes de cargar: {indice}".format(indice=self.indicePrograma))
        for index in range(0, progSize):
            inst = program.instructions[index]
            HARDWARE.memory.write(index + self.indicePrograma, inst)       
            log.logger.info("\n Cargando intruccion: programa: {} nroInst {} dir {} ints{}".format(program.name, index, index + self.indicePrograma,inst ))
        
        
        self.indicePrograma += progSize
        
        log.logger.info("\n Indice de programa despues de cargar: {indice}".format(indice=self.indicePrograma))
        log.logger.info("\n Program Size despues de cargar: {size}".format(size=progSize))
        return (self.indicePrograma - progSize)

class PCB():

    def pcb(self, program, baseDir, prioridad):
        self.pid = 0
        self.baseDir = baseDir
        self.pc = 0
        self.state = NEW
        self.path = program.name
        self.prioridad = prioridad


    def __repr__(self):
        return "pid {} baseDir {} pc {} state {} path {}".format(self.pid,self.baseDir,self.pc,self.state, self.path)

class PCBTable():

    def __init__(self):
        self.PCBs = {}
        self.pid = 0

    def add(self, pcb):
        self.PCBs[self.pid] = pcb
        pcb.pid = self.pid
        self.pid += 1
        log.logger.info("PCB new PCB en PCB {} table {}".format(pcb, self))

    def __repr__(self):
        return tabulate(enumerate(self.PCBs), tablefmt='psql')

    def pcbRunning(self):
        for k, v in self.PCBs.items():
            if (v.state == RUNNING):
                return v
        else:
            return None


    def todosPCBsTerminados(self):
        log.logger.info("todos procesos terminados? PCBTable {}".format(self))
        for k,v in self.PCBs.items():
            if (v.state != TERMINATED):        
                return False
        return True
    
class Dispatcher():

    def load(self, pcb):
        HARDWARE.cpu.pc = pcb.pc
        HARDWARE.mmu.baseDir = pcb.baseDir
        HARDWARE.timer.reset()
        pcb.state = RUNNING
    
    def save(self, pcb):
        pcb.pc = HARDWARE.cpu.pc
        HARDWARE.cpu.pc = -1
  

class Gantt():
   
    def __init__(self,kernel):
        self._ticks = []
        self._kernel = kernel
   
    def tick (self,tickNbr):
        log.logger.info("guardando informacion de los estados de los PCBs en el tick N {}".format(tickNbr))
        pcbYEstado = dict()
        pcbTable = self._kernel.pcbTable.PCBs
        for pid in pcbTable:
            pcb = pcbTable.get(pid)
            pcbYEstado[pid] = pcb.state
        self._ticks.append(pcbYEstado)
  
    def __repr__(self):
        return tabulate(enumerate(self._ticks), tablefmt='grid')

class ReadyQueue():
    def __init__(self):
       self.readyQueue= []
    
    def enqueue(self, pcb):
        pcb.state = READY
        self.readyQueue.append(pcb)
    
    def dequeue(self):
        return self.readyQueue.pop(0)

    def isEmpty(self):
        return len(self.readyQueue) == 0

    def insertarOrdenado(self, pcb):
        pcb.state = READY
        index = 0
        while index < len(self.readyQueue) and self.readyQueue[index].prioridad < pcb.prioridad:
            index += 1
        self.readyQueue.insert(index, pcb)
class Waiting_Queue():
    def __init__(self):
        self.waitingQ = []
        self.operations = []

    def enqueue(self, pcb, operation):
        self.waitingQ.append(pcb)
        self.operations.append(operation)

    
    def dequeue(self):
        pcb = self.waitingQ.pop(0)
        operation = self.operations.pop(0)
        pair = {'pcb': pcb, 'instruction': operation}
        return pair

    def isEmpty(self):
        return len(self.waitingQ) == 0


class Scheduler():
    
    def __init__(self, kernel):
        self.readyQ = ReadyQueue()
        self.kernel = kernel

    def add(self, pcb):
        if(self.kernel.pcbTable.pcbRunning() == None):
            self.kernel.dispatcher.load(pcb)
        else:
            self.addFromSon(pcb)
            
    def addFromSon(self, pcb):
        pass

    def nextPCB(self):
        return self.readyQ.dequeue()

    def isEmpty(self):
        return self.readyQ.isEmpty()

class FCFS(Scheduler):
        
    def addFromSon(self, pcb):
        self.readyQ.enqueue(pcb)

class PrioridadNoExpropiativa(Scheduler):

    def addFromSon(self, pcb):
        self.readyQ.insertarOrdenado(pcb)
class PrioridadExpropiativa(Scheduler):

    def addFromSon(self, pcb):
        pcbRunning = self.kernel.pcbTable.pcbRunning()
        if pcbRunning.prioridad > pcb.prioridad:
            self.kernel.dispatcher.save(pcbRunning)
            self.readyQ.insertarOrdenado(pcbRunning)
            self.kernel.dispatcher.load(pcb)
        
        else:
            self.readyQ.insertarOrdenado(pcb)
        

class RoundRobin(Scheduler):

    def addFromSon(self, pcb):
        self.readyQ.enqueue(pcb)

    def expropiarPCB(self,pcb):
        pcbRunning = self.kernel.pcbTable.pcbRunning()
        if not self.isEmpty() and pcbRunning != None:
            self.kernel.dispatcher.save(pcbRunning)
            self.add(pcbRunning)
            siguente=self.nextPCB()
            self.kernel.dispatcher.load(siguente)
