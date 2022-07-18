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
        
    def send_command(self,byte0,byte1,byte2,byte3) -> None:  # these are all ints
        '''
        Function:
            Send data to the ITLA
        Inputs:
            Four bytes
        '''
        msg = bytearray([byte0,byte1,byte2,byte3])
        self.conn.write(msg)

    def receive_response(self):
        '''
        Function:
            Receives a response from the ITLA
        Outputs:
            Four bytes
        '''
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

    def connect(self,port,baudrate=9600):
        '''
        Function:
            Establish serial connection with the ITLA at the maximum possible baud rate
        Inputs:
            Port to connect, initial baud rate
        Outputs:
            Errors if present
        '''
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
    
    def disconnect(self) -> None:
        '''
        Function:
            Close the serial connection with the ITLA
        '''
        self.conn.close()

    def ITLA(self,register,data,rw):
        '''
        Function:
            Prepare and send data to the ITLA
        Inputs:
            register address, data, read/write
        Outputs:
            The data returned by the ITLA (last two bytes)
        '''
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
            
    def AEA(self,bytes):
        outp=''
        while bytes>0:
            self.send_command(int(self.checksum(0,REG_AeaEar,0,0))*16,REG_AeaEar,0,0)
            test=self.receive_response()
            outp=outp+chr(test[2])
            outp=outp+chr(test[3])
            bytes=bytes-2
        return outp

    def set_power_dBm(self,power):
        '''
        Function:
            Set the power of the laser in dBm
        Inputs:
            Power in dBm
        '''
        data = 100*power  # data to send to the laser
        register = REG_Power
        set_power = self.ITLA(register,data,WRITE)
        return set_power
    
    def wait_until_no_operation(self):
        '''
        Function:
            Monitor the NOP register and halt operations until it is clear
        Returns:
            NOP register
        '''
        register = REG_Nop
        data = []
        while data != 16:
            data = self.ITLA(register,0,0)
            print(f'NOP returned: {data}')
        print(f'NOP returned {data}')
        return data


    def get_power_dBm(self):
        '''
        Function:
            Return the laser diode set power in dBm
        Return:
            Set laser diode power
        '''
        register = REG_Power
        return self.ITLA(register,0,READ)/100

    def turn_on(self):
        '''
        Function:
            Turn on the laser diode
        '''
        register = REG_Resena
        data = 8
        self.ITLA(register,data,WRITE)
        self.wait_until_no_operation()

    def turn_off(self):
        '''
        Function:
            Turn off the laser diode
        '''
        register = REG_Resena
        data = 0
        self.ITLA(register,data,WRITE)
        self.wait_until_no_operation()
    
    def set_wavelength_nm(self,wavelength):
        '''
        Function:
            Set the laser wavelength
        Inputs:
            Wavelength (in nm)
        '''
        self.set_frequency_THz(3e5/wavelength)

    def get_wavelength_nm(self):
        '''
        Function:
            Get the laser wavelength 
        Outputs:
            Laser frequency (in nm)
        '''
        return 3e5/self.get_frequency_THz()
    
    def set_frequency_THz(self,frequency):
        '''
        Function:
            Set the laser frequency 
        Inputs:
            Frequency (in THz)
        '''
        if frequency <= 196.25 and frequency >= 191.5:
            THz_register = REG_Fcf1
            GHz_register = REG_Fcf2
            data_THz = int(frequency)
            data_GHz = 10000*(frequency-data_THz)
            self.ITLA(THz_register,data_THz,WRITE)
            self.ITLA(GHz_register,data_GHz,WRITE)
            return
        raise RuntimeError('Invalid choice for frequency : %s' % frequency)

# TODO: figure out what happens if the specified frequency is out of range
#       and put checks in to stop whatever happens from happening


    def get_frequency_THz(self):
        '''
        Function:
            Return the frequency of the laser in THz
        Outputs:
            Frequency of the laser in THz
        '''
        THz_register = REG_Fcf1
        GHz_register = REG_Fcf2
        data_THz = self.ITLA(THz_register,0,READ)
        data_GHz = self.ITLA(GHz_register,0,READ)
        return data_THz + data_GHz/10000      
    
    def decode_response(self,response):
        '''
        Function:
            Decode the response sent by the ITLA
        Inputs:
            4-byte string sent from the ITLA
        Outputs:
            Data bits returned from the ITLA
        '''
        byte0 = response[0]
        byte1 = response[1]
        byte2 = response[2]
        byte3 = response[3]
        error_message = byte0 & 3  # extract bits 25 and 24
        if self.verbose:
            for byte,data in enumerate(response):
                print(f'byte{byte}: {hex(data)}')
        
        if error_message == 1: # execution error
            self.disconnect()
            print('error')

        return 256*byte2 + byte3

    def set_mode(self,mode) -> None:
        '''
        Function:
            Set the operating mode of the laser (see globals.py for modes)
        Inputs:
            Operating mode
        '''
        register = REG_Mode
        self.ITLA(register,mode,READ)
    
    def get_mode(self):
        '''
        Function:
            Return the operating mode of the laser (see globals.py for modes)
        Outputs:
            Operating mode
        '''
        register = REG_Mode
        return self.ITLA(register,0,READ)




if __name__ == '__main__':
    laser = ITLA()

    laser.connect('com7')

    laser.set_power_dBm(10)
    laser.set_frequency_THz(191.5)
    print('set power')
    laser.turn_on()
    print('turned on')
    time.sleep(5)
    laser.turn_off()
    laser.set_frequency_THz(196.25)
    print('changed frequency')
    laser.turn_on()
    print('turned on again')
    time.sleep(5)
    laser.turn_off()
    laser.disconnect()
    print('disconnected')

# TODO: mode setting
# TODO: current and temperature monitoring
# TODO: see if the threading and the queue are necessary
# TODO: add comments
# TODO: ensure the verbose flag is working the way I want
# TODO: reorganize functions in a way I like

# DONE:
# frequency checks work




             