ITLA_NOERROR=0x00
ITLA_EXERROR=0x01
ITLA_AEERROR=0x02
ITLA_CPERROR=0x03
ITLA_NRERROR=0x04
ITLA_CSERROR=0x05
ITLA_ERROR_SERPORT=0x01
ITLA_ERROR_SERBAUD=0x02

REG_Nop=0x00            # (read only) NOP / status
REG_Mfgr=0x02           # (read only) device type
REG_Model=0x03          # (read only) model ID
REG_Serial=0x04         # (read_only) serial number
REG_Release=0x06        # (read only) release info
REG_Gencfg=0x08         # general module configuration
REG_AeaEar=0x0B         # location accessed through AEA-EA and AEA-EAC
REG_Iocap=0x0D          # IO Capabilities
REG_Ear=0x10            # Location accessed through EA and EAC
REG_Dlconfig=0x14       # download configuration
REG_Dlstatus=0x15       # (read only) Download status
REG_Channel=0x30        # channel set-point
REG_Power=0x31          # power set point
REG_Resena=0x32         # device enable
REG_Grid=0x34           # grid
REG_Fcf1=0x35           # first channel frequency (THz part)
REG_Fcf2=0x36           # first channel frequency (GHz part)
REG_Oop=0x42            # (read only) optical output power
REG_Opsl=0x50           # (read only) power lower limit device capability
REG_Opsh=0x51           # (read only) power upper limit device capability
REG_Lfl1=0x52           # (read only) frequency lower limit device capability (THz part)
REG_Lfl2=0x53           # (read only) frequency lower limit device capability (GHz*10)
REG_Lfh1=0x54           # (read only) frequency upper limit device capability (THz part)
REG_Lfh2=0x55           # (read only) frequency upper limit device capability (GHz*10)
REG_Currents=0x57       # (read only, AEA) device currents
REG_Temps=0x58          # (read only, AEA) device temperatures (gain chip & case)
REG_Ftf=0x62            # fine tune frequency
REG_Mode=0x90           # select low noise mode
                        #   0: standard operation (with dither signal)
                        #   1: no-dither operation
                        #   2: whisper-mode operation
REG_PW=0xE0             # password to enable laser
                        #   W: provide password to the laser
                        #   R: provide 16 bit integer that will help Pure Photonics to calculate the password for you
REG_Csweepsena=0xE5     # (write only) start or stop the clean sweep feature
                        #   0: stop
                        #   1: start
REG_Csweepamp=0xE4      # range for the clean sweep feature in GHz
REG_Cscanamp=0xE4
REG_Cscanon=0xE5
REG_Csweepon=0xE5
REG_Csweepoffset=0xE6   # provide the offset during the clean sweep in units of 0.1 GHz with an offset of 200 GHz
                        # calculate the offset as: (read-out -2000) * 0.1 GHz
REG_Cscanoffset=0xE6
REG_Cscansled=0xF0
REG_Cscanf1=0xF1
REG_Cscanf2=0xF2
REG_CjumpTHz=0xEA
REG_CjumpGHz=0xEB
REG_CjumpSled=0xEC
REG_Cjumpon=0xED
REG_Cjumpoffset=0xE6

READ=0
WRITE=1