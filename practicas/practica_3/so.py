#!/usr/bin/env python

from hardware import *
import log
TERMINATED = "TERMINATED"
RUNNING = "RUNNING"
READY = "READY"
WAITING = "WAITING"
NEW = "NEW"


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
        self._instructions.append(instruction)

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
        self._waiting_queue = []
        self._currentPCB = None

    def runOperation(self, pcb, instruction):
        pair = {'pcb': pcb, 'instruction': instruction}
        # append: adds the element at the end of the queue
        self._waiting_queue.append(pair)
        # try to send the instruction to hardware's device (if is idle)
        self.__load_from_waiting_queue_if_apply()

    def getFinishedPCB(self):
        finishedPCB = self._currentPCB
        self._currentPCB = None
        self.__load_from_waiting_queue_if_apply()
        return finishedPCB

    def __load_from_waiting_queue_if_apply(self):
        if (len(self._waiting_queue) > 0) and self._device.is_idle:
            ## pop(): extracts (deletes and return) the first element in queue
            pair = self._waiting_queue.pop(0)
            #print(pair)
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
        log.logger.info("readyQueue {}".format(self.kernel.ready))
        procesoRunning = self.kernel.pcbTable1.pcbRunning()
        if (procesoRunning != None):
            procesoRunning.state = TERMINATED
            self.kernel.dispatcher.save(procesoRunning)
        if(len(self.kernel.ready) >= 1):
            next_pcb = self.kernel.ready.pop(0)
            self.kernel.dispatcher.load(next_pcb)
        elif(self.kernel.pcbTable1.todosPCBsTerminados()):##todos los procesos de pcbTable estan terminados
            HARDWARE.switchOff()

class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        operation = irq.parameters
        pcb = {'pc': HARDWARE.cpu.pc} # porque hacemos esto ???
        HARDWARE.cpu.pc = -1   ## dejamos el CPU IDLE
        self.kernel.ioDeviceController.runOperation(pcb, operation)
        log.logger.info(self.kernel.ioDeviceController)


class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self.kernel.ioDeviceController.getFinishedPCB()
        HARDWARE.cpu.pc = pcb['pc']
        log.logger.info(self.kernel.ioDeviceController)


# emulates the core of an Operative System
class Kernel():

    def __init__(self):
        self.loader = Loader()
        self.pcbTable1 = PCBTable()
        self.ready = []
        self.dispatcher = Dispatcher()
        ## setup interruption handlers
        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)


    @property
    def ioDeviceController(self):
        return self._ioDeviceController

    ## emulates a "system call" for programs execution
    def run(self, program):
        baseDir = self.loader.load(program)
        pcb1 = PCB()
        pcb1.pcb(program, baseDir)
        self.pcbTable1.table(pcb1)
        
        if(self.pcbTable1.pcbRunning() != None):
            self.ready.append(pcb1)
            pcb1.state = READY
        else:
            self.dispatcher.load(pcb1)
        
        log.logger.info("\n Executing program: {name}".format(name=program.name))
        log.logger.info("\n Diccionario: {pcbTable}".format(pcbTable=pcb1))
        log.logger.info(HARDWARE)



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

    def pcb(self, program, baseDir):
        self.pid = 0
        self.baseDir = baseDir
        self.pc = 0
        self.state = NEW
        self.path = program.name


    def __repr__(self):
        return "pid {} baseDir {} pc {} state {} path {}".format(self.pid,self.baseDir,self.pc,self.state, self.path)

class PCBTable():

    def __init__(self):
        self.PCBs = {}
        self.pid = 0

    def table(self, pcb):
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
        pcb.state = RUNNING
    
    def save(self, pcb):
        pcb.pc = HARDWARE.cpu.pc
        HARDWARE.cpu.pc = -1
  