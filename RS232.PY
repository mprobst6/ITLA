import serial
import time
import struct
import os
import os.path
import time
import sys
import threading
from globals import *

class ITLA:
    def __init__(self):
        self.latestregister=0
        self.tempport=0
        self.raybin=0
        self.queue=[]
        self.maxrowticket=0
        self._error=ITLA_NOERROR
        self.seriallock=0


    def stripString(input):
        outp=''
        input=str(input)
        teller=0
        while teller<len(input) and ord(input[teller])>47:
            outp=outp+input[teller]
            teller=teller+1
        return(outp)

    def ITLALastError():
        return(_error)

    def SerialLock():
        global seriallock
        return seriallock

    def SerialLockSet():
        global seriallock
        global queue
        seriallock=1
        
    def SerialLockUnSet():
        global seriallock
        global queue
        seriallock=0
        queue.pop(0)
        
    def checksum(byte0,byte1,byte2,byte3):
        bip8=(byte0&0x0f)^byte1^byte2^byte3   # & means AND, ^ means XOR
        bip4=((bip8&0xf0)>>4)^(bip8&0x0f)     # >> moves bits to the left, << moves to the right
        return bip4
        
    def send_command(sercon,byte0,byte1,byte2,byte3):  # these are all ints
        sercon.write(chr(byte0).encode('utf-8'))  # chr takes an integer and returns a charatecr who'se unicode point is num, an integer
        sercon.write(chr(byte1).encode('utf-8'))
        sercon.write(chr(byte2).encode('utf-8'))
        sercon.write(chr(byte3).encode('utf-8'))

    def receive_response(sercon):
        global _error,queue
        reftime=time.process_time()
        while sercon.inWaiting()<4:
            if time.process_time()>reftime+0.25:
                _error=ITLA_NRERROR
                return(0xFF,0xFF,0xFF,0xFF)
            time.sleep(0.0001)
        try:
            byte0=ord(sercon.read(1))
            byte1=ord(sercon.read(1))
            byte2=ord(sercon.read(1))
            byte3=ord(sercon.read(1))
        except:
            print(f'problem with serial communication. queue[0] = {queue}')
            byte0=0xFF
            byte1=0xFF
            byte2=0xFF
            byte3=0xFF
        if self.checksum(byte0,byte1,byte2,byte3)==byte0>>4:
            _error=byte0&0x03
            return(byte0,byte1,byte2,byte3)
        else:
            _error=ITLA_CSERROR
            return(byte0,byte1,byte2,byte3)       

    def receive_simple_response(sercon):
        global _error,CoBrite
        reftime=time.process_time()
        while sercon.inWaiting()<4:
            if time.process_time()>reftime+0.25:
                _error=ITLA_NRERROR
                return(0xFF,0xFF,0xFF,0xFF)
            time.sleep(0.0001)
        byte0=ord(sercon.read(1))
        byte1=ord(sercon.read(1))
        byte2=ord(sercon.read(1))
        byte3=ord(sercon.read(1)) 

    def ITLAConnect(port,baudrate=9600):
        global CoBrite
        reftime=time.process_time()
        connected=False
        try:
            conn = serial.Serial(port,baudrate, timeout=1)
        except serial.SerialException:
            return(ITLA_ERROR_SERPORT)
        baudrate2=4800
        while baudrate2<115200:
            self.ITLA(conn,REG_Nop,0,0)
            if self.ITLALastError() != ITLA_NOERROR:
                #go to next baudrate
                if baudrate2==4800:baudrate2=9600
                elif baudrate2==9600: baudrate2=19200
                elif baudrate2==19200: baudrate2=38400
                elif baudrate2==38400:baudrate2=57600
                elif baudrate2==57600:baudrate2=115200
                conn.close()
                conn = serial.Serial(port,baudrate2 , timeout=1)            
            else:
                return(conn)
        conn.close()
        return(ITLA_ERROR_SERBAUD)

    def ITLA(sercon,register,data,rw):
        global latestregister
        lock=threading.Lock()
        lock.acquire()
        global queue
        global maxrowticket
        rowticket=maxrowticket+1
        maxrowticket=maxrowticket+1
        queue.append(rowticket)
        lock.release()
        while queue[0] != rowticket:
            rowticket=rowticket
        if rw==0: # read
            byte2=int(data/256)
            byte3=int(data-byte2*256)
            latestregister=register
            self.send_command(sercon,int(checksum(0,register,byte2,byte3))*16,register,byte2,byte3)
            test=self.receive_response(sercon)
            b0=test[0]
            b1=test[1]
            b2=test[2]
            b3=test[3]
            if (b0&0x03)==0x02:
                test=AEA(sercon,b2*256+b3)
                lock.acquire()
                queue.pop(0)
                lock.release()
                return test
            lock.acquire()
            queue.pop(0)
            lock.release()
            return b2*256+b3
        else: # write
            byte2=int(data/256)
            byte3=int(data-byte2*256)
            self.send_command(sercon,int(checksum(1,register,byte2,byte3))*16+1,register,byte2,byte3)
            test=self.receive_response(sercon)
            lock.acquire()
            queue.pop(0)
            lock.release()
            return(test[2]*256+test[3])

    def ITLA_send_only(sercon,register,data,rw):
        global latestregister
        global queue
        global maxrowticket
        rowticket=maxrowticket+1
        maxrowticket=maxrowticket+1
        queue.append(rowticket)
        while queue[0] != rowticket:
            time.sleep(.1)
        self.SerialLockSet()
        if rw==0:
            latestregister=register
            self.send_command(sercon,int(checksum(0,register,0,0))*16,register,0,0)
            self.receive_simple_response(sercon)
            self.SerialLockUnSet()
        else:
            byte2=int(data/256)
            byte3=int(data-byte2*256)
            self.send_command(sercon,int(checksum(1,register,byte2,byte3))*16+1,register,byte2,byte3)
            self.receive_simple_response(sercon)
            self.SerialLockUnSet()
            
    def AEA(sercon,bytes):
        outp=''
        while bytes>0:
            self.send_command(sercon,int(checksum(0,REG_AeaEar,0,0))*16,REG_AeaEar,0,0)
            test=self.receive_response(sercon)
            outp=outp+chr(test[2])
            outp=outp+chr(test[3])
            bytes=bytes-2
        return outp



    def ITLAFWUpgradeStart(sercon,raydata,salvage=0):
        global tempport,raybin
        #set the baudrate to maximum and reconfigure the serial connection
        if salvage==0:
            ref=stripString(ITLA(sercon,REG_Serial,0,0))
            if len(ref)<5:
                print('problems with communication before start FW upgrade')
                return(sercon,'problems with communication before start FW upgrade')
            self.ITLA(sercon,REG_Resena,0,1)
        self.ITLA(sercon,REG_Iocap,64,1) #bits 4-7 are 0x04 for 115200 baudrate
        #validate communication with the laser
        tempport=sercon.portstr
        sercon.close()
        sercon = serial.Serial(tempport, 115200, timeout=1)
        if stripString(self.ITLA(sercon,REG_Serial,0,0)) != ref:
            return(sercon,'After change baudrate: serial discrepancy found. Aborting. '+str(stripString(ITLA(sercon,REG_Serial,0,0)))+' '+str( params.serial))
        #load the ray file
        raybin=raydata
        if (len(raybin)&0x01):raybin.append('\x00')
        self.ITLA(sercon,REG_Dlconfig,2,1)  #first do abort to make sure everything is ok
        #print self.ITLALastError()
        if self.ITLALastError() != ITLA_NOERROR:
            return( sercon,'After dlconfig abort: error found. Aborting. ' + str(self.ITLALastError()))
        #initiate the transfer; INIT_WRITE=0x0001; TYPE=0x1000; RUNV=0x0000
        #temp=self.ITLA(sercon,REG_Dlconfig,0x0001 ^ 0x1000 ^ 0x0000,1)
        #check temp for the correct feedback
        self.ITLA(sercon,REG_Dlconfig,3*16*256+1,1) # initwrite=1; type =3 in bits 12:15
        #print self.ITLALastError()
        if self.ITLALastError() != ITLA_NOERROR:
            return(sercon,'After dlconfig init_write: error found. Aborting. '+str(self.ITLALastError() ))
        return(sercon,'')

    def ITLAFWUpgradeWrite(sercon,count):
        global tempport,raybin
        #start writing bits
        teller=0
        while teller<count:
            ITLA_send_only(sercon,REG_Ear,struct.unpack('>H',raybin[teller:teller+2])[0],1)
            teller=teller+2
        raybin=raybin[count:]
        #write done. clean up
        return('')

    def ITLAFWUpgradeComplete(sercon):
        global tempport,raybin
        time.sleep(0.5)
        sercon.flushInput()
        sercon.flushOutput()
        ITLA(sercon,REG_Dlconfig,4,1) # done (bit 2)
        if self.ITLALastError() != ITLA_NOERROR:
            return(sercon,'After dlconfig done: error found. Aborting. '+str(self.ITLALastError()))
        #init check
        ITLA(sercon,REG_Dlconfig,16,1) #init check bit 4
        if self.ITLALastError()==ITLA_CPERROR:
            while (ITLA(sercon,REG_Nop,0,0)&0xff00)>0:
                time.sleep(0.5)
        elif self.ITLALastError() != ITLA_NOERROR:
            return(sercon,'After dlconfig done: error found. Aborting. '+str(self.ITLALastError() ))
        #check for valid=1
        temp=ITLA(sercon,REG_Dlstatus,0,0)
        if (temp&0x01==0x00):
            return(sercon,'Dlstatus not good. Aborting. ')           
        #write concluding dlconfig
        ITLA(sercon,REG_Dlconfig,3*256+32, 1) #init run (bit 5) + runv (bit 8:11) =3
        if self.ITLALastError() != ITLA_NOERROR:
            return(sercon, 'After dlconfig init run and runv: error found. Aborting. '+str(self.ITLALastError()))
        time.sleep(1)
        #set the baudrate to 9600 and reconfigure the serial connection
        ITLA(sercon,REG_Iocap,0,1) #bits 4-7 are 0x0 for 9600 baudrate
        sercon.close()
        #validate communication with the laser
        sercon = serial.Serial(tempport, 9600, timeout=1)
        ref=stripString(ITLA(sercon,REG_Serial,0,0))
        if len(ref)<5:
            return( sercon,'After change back to 9600 baudrate: serial discrepancy found. Aborting. '+str(stripString(ITLA(sercon,REG_Serial,0,0)))+' '+str( params.serial))
        return(sercon,'')

    def ITLASplitDual(input,rank):
        teller=rank*2
        return(ord(input[teller])*256+ord(input[teller+1]))

    def set_power_dBm(sercon,power):
        data = 100*power  # data to send to the laser

    


             