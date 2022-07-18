import serial
import time
import struct
import os
import os.path
import sys
import threading
from globals import *
from response import *

class ITLA:
    def __init__(self):
        self.latestregister=0
        self.tempport=0
        self.raybin=0
        self.queue=[]
        self.maxrowticket=0
        self._error=ITLA_NOERROR
        self.seriallock=0
        self.conn = []
        self.verbose = False


    def stripString(self,input):
        outp=''
        input=str(input)
        teller=0
        while teller<len(input) and ord(input[teller])>47:
            outp=outp+input[teller]
            teller=teller+1
        return(outp)

    def ITLALastError(self):
        return(self._error)

    def SerialLock(self):
        return self.seriallock

    def SerialLockSet(self):
        self.seriallock
        self.queue
        self.seriallock=1
        
    def SerialLockUnSet(self):
        self.seriallock
        self.queue
        self.seriallock=0
        self.queue.pop(0)
        
    def checksum(self,byte0,byte1,byte2,byte3):
        bip8=(byte0&0x0f)^byte1^byte2^byte3   # & means AND, ^ means XOR
        bip4=((bip8&0xf0)>>4)^(bip8&0x0f)     # >> moves bits to the left, << moves to the right
        return bip4
        
    def send_command(self,byte0,byte1,byte2,byte3):  # these are all ints
        msg = bytearray([byte0,byte1,byte2,byte3])
        self.conn.write(msg)

    def receive_response(self):
        reftime=time.process_time()
        while self.conn.inWaiting()<4:
            if time.process_time()>reftime+0.25:
                _error=ITLA_NRERROR
                return(0xFF,0xFF,0xFF,0xFF)
            time.sleep(0.0001)
        try:
            byte0=ord(self.conn.read(1))
            byte1=ord(self.conn.read(1))
            byte2=ord(self.conn.read(1))
            byte3=ord(self.conn.read(1))
        except:
            print(f'problem with serial communication. queue[0] = {self.queue}')
            byte0=0xFF
            byte1=0xFF
            byte2=0xFF
            byte3=0xFF
        if self.checksum(byte0,byte1,byte2,byte3)==byte0>>4:
            self._error=byte0&0x03
            return(byte0,byte1,byte2,byte3)
        else:
            self._error=ITLA_CSERROR
            return(byte0,byte1,byte2,byte3)       

    def receive_simple_response(self):
        reftime=time.process_time()
        while self.conn.inWaiting()<4:
            if time.process_time()>reftime+0.25:
                self._error=ITLA_NRERROR
                return(0xFF,0xFF,0xFF,0xFF)
            time.sleep(0.0001)
        byte0=ord(self.conn.read(1))
        byte1=ord(self.conn.read(1))
        byte2=ord(self.conn.read(1))
        byte3=ord(self.conn.read(1)) 

    def connect(self,port,baudrate=9600):
        reftime=time.process_time()
        try:
            self.conn = serial.Serial(port,baudrate, timeout=1)
        except serial.SerialException:
            return(ITLA_ERROR_SERPORT)
        baudrate2=4800
        while baudrate2<115200:
            self.ITLA(REG_Nop,0,0)
            if self.ITLALastError() != ITLA_NOERROR:
                #go to next baudrate
                if baudrate2==4800:baudrate2=9600
                elif baudrate2==9600: baudrate2=19200
                elif baudrate2==19200: baudrate2=38400
                elif baudrate2==38400:baudrate2=57600
                elif baudrate2==57600:baudrate2=115200
                self.conn.close()
                self.conn = serial.Serial(port,baudrate2 , timeout=1)            
            else:
                return
        self.conn.close()
        return(ITLA_ERROR_SERBAUD)
    
    def disconnect(self):
        self.conn.close()

    def ITLA(self,register,data,rw):
        lock=threading.Lock()
        lock.acquire()
        rowticket=self.maxrowticket+1
        self.maxrowticket=self.maxrowticket+1
        self.queue.append(rowticket)
        lock.release()
        while self.queue[0] != rowticket:
            rowticket=rowticket
        if rw==0: # read
            byte2=int(data/256)
            byte3=int(data-byte2*256)
            latestregister=register
            self.send_command(int(self.checksum(0,register,byte2,byte3))*16,register,byte2,byte3)
            response = self.receive_response()
            self.decode_response(response)
            b0 = response[0]
            b1 = response[1]
            b2 = response[2]
            b3 = response[3]
            if (b0&0x03)==0x02:
                response=AEA(b2*256+b3)
                lock.acquire()
                self.queue.pop(0)
                lock.release()
                return 256*b2 + b3
            lock.acquire()
            self.queue.pop(0)
            lock.release()
            return 256*b2 + b3
        else: # write
            byte2=int(data/256)
            byte3=int(data-byte2*256)
            byte0 = int(self.checksum(1,register,byte2,byte3))*16+1
            if self.verbose:
                print('\nWriting the following command')
                print(f'byte0: {hex(byte0)}')
                print(f'register: {hex(register)}') 
                print(f'byte2: {hex(byte2)}')
                print(f'byte3: {hex(byte3)}')            
            self.send_command(byte0,register,byte2,byte3)
            response = self.receive_response()
            self.decode_response(response)
            lock.acquire()
            self.queue.pop(0)
            lock.release()
            return 256*response[2] + response[3]

    def ITLA_send_only(self,register,data,rw):
        rowticket=self.maxrowticket+1
        self.maxrowticket=maxrowticket+1
        self.queue.append(rowticket)
        while self.queue[0] != rowticket:
            time.sleep(.1)
        self.SerialLockSet()
        if rw==0:
            self.latestregister=register
            self.send_command(int(self.checksum(0,register,0,0))*16,register,0,0)
            self.receive_simple_response()
            self.SerialLockUnSet()
        else:
            byte2=int(data/256)
            byte3=int(data-byte2*256)
            self.send_command(int(self.checksum(1,register,byte2,byte3))*16+1,register,byte2,byte3)
            self.receive_simple_response()
            self.SerialLockUnSet()
            
    def AEA(self,bytes):
        outp=''
        while bytes>0:
            self.send_command(int(self.checksum(0,REG_AeaEar,0,0))*16,REG_AeaEar,0,0)
            test=self.receive_response()
            outp=outp+chr(test[2])
            outp=outp+chr(test[3])
            bytes=bytes-2
        return outp


    def ITLAFWUpgradeStart(self,raydata,salvage=0):
        #set the baudrate to maximum and reconfigure the serial connection
        if salvage==0:
            ref=self.stripString(ITLA(REG_Serial,0,0))
            if len(ref)<5:
                print('problems with communication before start FW upgrade')
                return(self.conn,'problems with communication before start FW upgrade')
            self.ITLA(REG_Resena,0,1)
        self.ITLA(REG_Iocap,64,1) #bits 4-7 are 0x04 for 115200 baudrate
        #validate communication with the laser
        self.tempport=self.conn.portstr
        self.conn.close()
        self.conn = serial.Serial(tempport, 115200, timeout=1)
        if self.stripString(self.ITLA(REG_Serial,0,0)) != ref: #TODO fix this since I modified the ITLA return
            return(self.conn,'After change baudrate: serial discrepancy found. Aborting. '+str(self.stripString(ITLA(self.conn,REG_Serial,0,0)))+' '+str( params.serial))
        #load the ray file
        self.raybin=raydata
        if (len(self.raybin)&0x01):self.raybin.append('\x00')
        self.ITLA(self.conn,REG_Dlconfig,2,1)  #first do abort to make sure everything is ok
        #print self.ITLALastError()
        if self.ITLALastError() != ITLA_NOERROR:
            return( self.conn,'After dlconfig abort: error found. Aborting. ' + str(self.ITLALastError()))
        #initiate the transfer; INIT_WRITE=0x0001; TYPE=0x1000; RUNV=0x0000
        #temp=self.ITLA(self.conn,REG_Dlconfig,0x0001 ^ 0x1000 ^ 0x0000,1)
        #check temp for the correct feedback
        self.ITLA(self.conn,REG_Dlconfig,3*16*256+1,1) # initwrite=1; type =3 in bits 12:15
        #print self.ITLALastError()
        if self.ITLALastError() != ITLA_NOERROR:
            return(self.conn,'After dlconfig init_write: error found. Aborting. '+str(self.ITLALastError() ))
        return(self.conn,'')

    def ITLAFWUpgradeWrite(self,count):
        #start writing bits
        teller=0
        while teller<count:
            ITLA_send_only(REG_Ear,struct.unpack('>H',self.raybin[teller:teller+2])[0],1)
            teller=teller+2
        self.raybin=self.raybin[count:]
        #write done. clean up
        return('')

    def ITLAFWUpgradeComplete(self):
        time.sleep(0.5)
        self.conn.flushInput()
        self.conn.flushOutput()
        ITLA(REG_Dlconfig,4,1) # done (bit 2)
        if self.ITLALastError() != ITLA_NOERROR:
            return(self.conn,'After dlconfig done: error found. Aborting. '+str(self.ITLALastError()))
        #init check
        ITLA(REG_Dlconfig,16,1) #init check bit 4
        if self.ITLALastError()==ITLA_CPERROR:
            while (ITLA(REG_Nop,0,0)&0xff00)>0:
                time.sleep(0.5)
        elif self.ITLALastError() != ITLA_NOERROR:
            return(self.conn,'After dlconfig done: error found. Aborting. '+str(self.ITLALastError() ))
        #check for valid=1
        temp=ITLA(REG_Dlstatus,0,0)
        if (temp&0x01==0x00):
            return(self.conn,'Dlstatus not good. Aborting. ')           
        #write concluding dlconfig
        ITLA(REG_Dlconfig,3*256+32, 1) #init run (bit 5) + runv (bit 8:11) =3
        if self.ITLALastError() != ITLA_NOERROR:
            return(self.conn, 'After dlconfig init run and runv: error found. Aborting. '+str(self.ITLALastError()))
        time.sleep(1)
        #set the baudrate to 9600 and reconfigure the serial connection
        ITLA(REG_Iocap,0,1) #bits 4-7 are 0x0 for 9600 baudrate
        self.conn.close()
        #validate communication with the laser
        self.conn = serial.Serial(self.tempport, 9600, timeout=1)
        ref = self.stripString(ITLA(REG_Serial,0,0))
        if len(ref)<5:
            return( self.conn,'After change back to 9600 baudrate: serial discrepancy found. Aborting. '+str(stripString(ITLA(self.conn,REG_Serial,0,0)))+' '+str( params.serial))
        return(self.conn,'')

    def ITLASplitDual(self,input,rank):
        teller=rank*2
        return(ord(input[teller])*256+ord(input[teller+1]))

    def set_power_dBm(self,power):
        data = 100*power  # data to send to the laser
        register = REG_Power
        set_power = self.ITLA(register,data,WRITE)
        return set_power
    
    def wait_until_no_operation(self):
        register = REG_Nop
        data = []
        while data != 16:
            data = self.ITLA(register,0,0)
            print(f'NOP returned: {data}')
        print(f'NOP returned {data}')
        return data


    def get_power_dBm(self):
        register = REG_Power
        return self.ITLA(register,0,READ)/100

    def turn_on(self):
        register = REG_Resena
        data = 8
        self.ITLA(register,data,WRITE)
        self.wait_until_no_operation()

    def turn_off(self):
        register = REG_Resena
        data = 0
        self.ITLA(register,data,WRITE)
        self.wait_until_no_operation()
    
    def set_wavelength_nm(self,wavelength):
        self.set_frequency_THz(3e5/wavelength)

    def get_wavelength_nm(self):
        return 3e5/self.get_frequency_THz()
    
    def set_frequency_THz(self,frequency):
        THz_register = REG_Fcf1
        GHz_register = REG_Fcf2
        data_THz = int(frequency)
        data_GHz = 10000*(frequency-data_THz)
        self.ITLA(THz_register,data_THz,WRITE)
        self.ITLA(GHz_register,data_GHz,WRITE)

# TODO: figure out what happens if the specified frequency is out of range
#       and put checks in to stop whatever happens from happening


    def get_frequency_THz(self):
        THz_register = REG_Fcf1
        GHz_register = REG_Fcf2
        data_THz = self.ITLA(THz_register,0,READ)
        data_GHz = self.ITLA(GHz_register,0,READ)
        return data_THz + data_GHz/10000      
    
    def decode_response(self,response):
        byte0 = response[0]
        byte1 = response[1]
        byte2 = response[2]
        byte3 = response[3]
        error_message = byte0 & 3  # extract bits 25 and 24
        if self.verbose:
            for byte,data in enumerate(response):
                print(f'byte{byte}: {hex(data)}')
        
        if error_message == 1: # execution error
            self.ITLA_disconnect()
            print('error')

        return 256*byte2 + byte3

# TODO: mode setting may or may not be working.
# It does not return the mode that it is in, so I have no way to verify right now if it is working
    def set_mode(self,mode):
        register = REG_Mode
        self.ITLA(register,mode,READ)
    
    def get_mode(self):
        register = REG_Mode
        return self.ITLA(register,0,READ)




if __name__ == '__main__':
    laser = ITLA()

    laser.connect('com7')

    laser.set_power_dBm(10)
    laser.set_wavelength_nm(1530)
    print('set power')
    laser.turn_on()
    print('turned on')
    time.sleep(5)
    laser.turn_off()
    laser.set_wavelength_nm(1560)
    print('changed frequency')
    laser.turn_on()
    print('turned on again')
    time.sleep(5)
    laser.turn_off()
    laser.disconnect()
    print('disconnected')

    # seem to be sending ok, receiving si the sisue
    # make the serial connection an attribute so I don't have to pass it in every time
    # decode_response takes the full response to decode error messages (and returns data)
    # ITLA just returns the data




             