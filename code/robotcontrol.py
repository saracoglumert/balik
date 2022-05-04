from distutils.log import debug
from pickle import TRUE
import serial
import time
from threading import Thread

class _Data:
    SYNC            = False
    MOTOR_STATUS    = False
    XPOS            = 0     #mms
    YPOS            = 0     #mms
    THPOS           = 0     #degs
    LVEL            = 0     #mm/s
    RVEL            = 0     #mm/s
    BATTERY         = 0     #Volts
    SONARCOUNT      = 0
    SONAR1          = 0
    SONAR2          = 0
    
    COMMAND         = []
    LASTCOMMAND     = ""

class _Config:
    SERIAL_PORT                 = "/dev/ttyUSB0"
    SERIAL_BAUDRATE             = 9600
    SERIAL_TIMEOUT              = 0.1

    HEARTRATE                   = 0.1
    WAIT                        = 0.5

    DEBUG                       = TRUE

    PACKAGE_ENDIAN              = "little"
    PACKAGE_HEADER1             = 'FA'
    PACKAGE_HEADER2             = 'FB'
    PACKAGE_ARG_TYPE_POSITIVE   = "3B"
    PACKAGE_ARG_TYPE_NEGATIVE   = "1B"
    PACKAGE_BYTE_COUNT          = "06"
    PACKAGE_PREFIX              = "0x"
    PACKAGE_SYNC1               = b'\xFA\xFB\x03\x00\x00\x00'
    PACKAGE_SYNC2               = b'\xFA\xFB\x03\x01\x00\x01'
    PACKAGE_SYNC3               = b'\xFA\xFB\x03\x02\x00\x02'
    PACKAGE_OPEN                = b'\xFA\xFB\x03\x01\x00\x01'
    PACKAGE_PULSE               = b'\xFA\xFB\x03\x00\x00\x00'
    PACKAGE_ENABLE              = b'\xFA\xFB\x06\x04\x3B\x00\x01\x10\x84'
    PACKAGE_DISABLE             = b'\xFA\xFB\x06\x04\x3B\x00\x00\x10\x83'
    PACKAGE_STOP                = b'\xFA\xFB\x03\x29\x00\x29'
    PACKAGE_ESTOP               = b'\xFA\xFB\x03\x55\x00\x55'
    PACKAGE_SONAR_ENABLE        = b'\xFA\xFB\x06\x28\x3B\x00\x01\x28\x3C'
    PACKAGE_SONAR_DISABLE       = b'\xFA\xFB\x06\x28\x3B\x00\x01\x28\x3B'
    PACKAGE_CLOSE               = b'\xFA\xFB\x03\x02\x00\x02'
    PACKAGE_SETORIGIN           = b'\xFA\xFB\x03\x07\x00\x07'

    WAITCOEFF_TRANSLATION       = 2
    WAITCOEFF_ROTATION          = 3

    CONSOLE_CURSOR              = ">"
    CONSOLE_TRANSLATE_FORWARD   = "TF"
    CONSOLE_TRANSLATE_BACKWARD  = "TB"
    CONSOLE_ROTATE_RIGHT        = "RR"
    CONSOLE_ROTATE_LEFT         = "RL"
    CONSOLE_WAIT                = "WW"
    CONSOLE_READ                = "STAT"
    CONSOLE_CLOSE               = "CLOSE"
    CONSOLE_TEST                = "TEST"
    CONSOLE_SEPERATOR           = ":"
    CONSOLE_ERROR               = "Argument error."
    CONSOLE_STATUS_MESSAGE      = "XPOS          : {}\nYPOS          : {}\nTHPOS         : {}\nRVEL          : {}\nLVEL          : {}\nBATTERY       : {}\nSONARCOUNT    : {}\nSONARS        : {},{}"
    
    DEBUG_CHECKSUM_INPUT        = "Input checksum   : {}"
    DEBUG_CHECKSUM_OUTPUT       = "Output checksum  : {}"
    DEBUG_PACKAGE_CONTENT       = "Package content  : {}"
    DEBUG_XOR                   = "Unexpected case. Use XOR."
    DENEMEPAKET                 = b'\xFA\xFB\x06\x08\x3B\x00\xFF\x09\x3A'
    
class _Tools:
    @staticmethod
    def IsAnyThreadAlive(threads):
        return True in [t.is_alive() for t in threads]

    @staticmethod
    def Package_Command(input):
        temp = str(hex(input))
        temp = temp.replace(_Config.PACKAGE_PREFIX,"")
        if (len(temp) == 1):
            temp = "0" + temp
        temp = temp.upper()
        return temp
    
    @staticmethod
    def Package_Arguments(input):
        temp2 = []
        temp = input.to_bytes(2,_Config.PACKAGE_ENDIAN)
        for element in temp:
            temp3 = hex(element).replace(_Config.PACKAGE_PREFIX,"").upper()
            if (len(temp3) == 1):
                temp3 = "0" + temp3
            temp2.append(temp3)
        return temp2

    @staticmethod
    def Package_Checksum(input):
        if(_Config.DEBUG):
            print(_Config.DEBUG_CHECKSUM_INPUT.format(str(input)))
        if (len(input) % 2 == 0 ):
            temp1 = int(input[0]+input[1],16)
            temp2 = int(input[2]+input[3],16)
            temp = hex(temp1 + temp2)
            temp = temp.replace(_Config.PACKAGE_PREFIX,"")
            temp = temp.upper()
            if (len(temp) % 2 == 1):
                temp = "0" + temp
            temp2 = [temp[i:i+2] for i in range(0, len(temp), 2)]
            if(_Config.DEBUG):
                print(_Config.DEBUG_CHECKSUM_OUTPUT.format(str(temp2[-2:])))
            return temp2[-2:]
        else:
            if(_Config.DEBUG):
                print(_Config.DEBUG_XOR)
            return 0
    
    @staticmethod
    def Package(command,argument):
        temp = []

        temp.append(_Config.PACKAGE_HEADER1)
        temp.append(_Config.PACKAGE_HEADER2)
        temp.append(_Config.PACKAGE_BYTE_COUNT)
        temp.append(_Tools.Package_Command(command))
        
        if (argument > 0):
            temp.append(_Config.PACKAGE_ARG_TYPE_POSITIVE)
        if (argument < 0):
            temp.append(_Config.PACKAGE_ARG_TYPE_NEGATIVE)
        
        temp += _Tools.Package_Arguments(abs(argument))
        temp += _Tools.Package_Checksum(temp[3:])

        if(_Config.DEBUG):
            print(_Config.DEBUG_PACKAGE_CONTENT.format(str(temp)))

        return bytes.fromhex("".join(temp))

    @staticmethod
    def CalculateDuration(input):
        header = input[:2]
        argument = int(input[2:])
        if (header[0] == "T"):
            return argument*_Config.WAITCOEFF_TRANSLATION
        elif (header[0] == "R"):
            return argument*_Config.WAITCOEFF_ROTATION
        else:
            return 0

class _Robot:
    @staticmethod
    def Init():
        SerialConnection.write(_Config.PACKAGE_SYNC1)
        SerialConnection.write(_Config.PACKAGE_SYNC2)
        SerialConnection.write(_Config.PACKAGE_SYNC3)
        print("[OK] Sync")
        time.sleep(_Config.WAIT)
        SerialConnection.write(_Config.PACKAGE_OPEN)
        _Data.SYNC = True
        print("[OK] Connection Open")
        time.sleep(_Config.WAIT)
        SerialConnection.write(_Tools.Package(28,1))
        print("[OK] Sonars Enabled")
        time.sleep(_Config.WAIT)
        SerialConnection.write(_Tools.Package(4,1))
        print("[OK] Motors Enabled")
        _Data.MOTOR_STATUS = True

    @staticmethod
    def Heartbeat():
        while True:
            SerialConnection.write(_Config.PACKAGE_PULSE)
            time.sleep(_Config.HEARTRATE)
    
    @staticmethod
    def Read():
        while True:
            temp = b''
            while True:
                current = SerialConnection.read(1)
                if (current == bytes.fromhex(_Config.PACKAGE_HEADER1)):
                    break
                else:
                    temp += current
            try:
                _Data.XPOS = int.from_bytes(temp[3:5], _Config.PACKAGE_ENDIAN,signed=True)*-1
                _Data.YPOS = int.from_bytes(temp[5:7], _Config.PACKAGE_ENDIAN,signed=True)*-1
                _Data.THPOS = int.from_bytes(temp[7:9], _Config.PACKAGE_ENDIAN,signed=True)*-1
                _Data.LVEL = int.from_bytes(temp[9:11], _Config.PACKAGE_ENDIAN,signed=True)*-1
                _Data.RVEL = int.from_bytes(temp[11:13], _Config.PACKAGE_ENDIAN,signed=True)*-1
                _Data.BATTERY = temp[13]/10
                _Data.SONARCOUNT = temp[21]
                _Data.SONAR1 = int.from_bytes(temp[24], _Config.PACKAGE_ENDIAN,signed=False)
                _Data.SONAR2 = int.from_bytes(temp[25], _Config.PACKAGE_ENDIAN,signed=False)
            except:
                pass

    @staticmethod
    def SetOrigin():
        SerialConnection.write(_Config.PACKAGE_SETORIGIN)

    @staticmethod
    def SetTranslationA(input):
        pkg = _Tools.Package(5,input)
        SerialConnection.write(pkg)

    @staticmethod
    def SetTranslationV(input):
        pkg = _Tools.Package(6,input)
        SerialConnection.write(pkg)

    @staticmethod
    def SetRotationV(input):
        pkg = _Tools.Package(10,input)
        SerialConnection.write(pkg)

    @staticmethod
    def Translate(input):
        pkg = _Tools.Package(8,input)
        SerialConnection.write(pkg)
        
    @staticmethod
    def Rotate(input):
        pkg = _Tools.Package(13,input)
        SerialConnection.write(pkg)

    @staticmethod
    def Execute(input):
        if(_HMI.IsValid(input) == 1):
            header = input[:2]
            argument = int(input[2:])
            if(header ==_Config.CONSOLE_TRANSLATE_FORWARD):
                _Robot.Translate(argument)
            if(header == _Config.CONSOLE_TRANSLATE_BACKWARD):
                _Robot.Translate(-argument)
            if(header == _Config.CONSOLE_ROTATE_RIGHT):
                _Robot.Rotate(-argument)
            if(header == _Config.CONSOLE_ROTATE_LEFT):
                _Robot.Rotate(argument)
            _Data.LASTCOMMAND = input
        elif(_HMI.IsValid(input) == 2):
            cmds = input.split(_Config.CONSOLE_SEPERATOR)
            for cmd in cmds:
                header = cmd[:2]
                if (header == _Config.CONSOLE_WAIT and len(cmd[2:]) == 0):
                    argument = _Tools.CalculateDuration(_Data.LASTCOMMAND)
                else:
                    argument = int(cmd[2:])
                if(header ==_Config.CONSOLE_TRANSLATE_FORWARD):
                    _Robot.Translate(argument)
                if(header == _Config.CONSOLE_TRANSLATE_BACKWARD):
                    _Robot.Translate(-argument)
                if(header == _Config.CONSOLE_ROTATE_RIGHT):
                    _Robot.Rotate(-argument)
                if(header == _Config.CONSOLE_ROTATE_LEFT):
                    _Robot.Rotate(argument)
                if(header == _Config.CONSOLE_WAIT):
                    time.sleep(argument/1000)
                _Data.LASTCOMMAND = cmd


    @staticmethod
    def Test():
        _Robot.Translate(1000)
        _Robot.Rotate(45)
        _Robot.Translate(250)
        _Robot.Rotate(-30)  
        
class _HMI:
    @staticmethod
    def IsValid(input):
        input = input.upper()
        if _Config.CONSOLE_SEPERATOR in input:
            input2 = input.split(_Config.CONSOLE_SEPERATOR)
            temp = 1
            for element in input2:
                header = element[:2]
                argument = element[2:]
                if (argument.isnumeric()):
                    temp = temp*1
                elif(header == _Config.CONSOLE_WAIT and len(argument) == 0):
                    temp = temp*1
                else:
                    temp = temp*(-1)
            if (temp == 1):
                return 2
            else:
                return 0
        elif _Config.CONSOLE_SEPERATOR not in input:
            argument = input[2:]
            if (argument.isnumeric()):
                return 1
            else:
                return 0
    
    @staticmethod
    def Console():
        while True:
            currentinput = input(_Config.CONSOLE_CURSOR)
            currentinput = currentinput.upper()
            if(_HMI.IsValid(currentinput)):
                _Robot.Execute(currentinput)
            else:
                if (currentinput == _Config.CONSOLE_READ):
                    print(_Config.CONSOLE_STATUS_MESSAGE.format(_Data.XPOS,_Data.YPOS,_Data.THPOS,_Data.RVEL,_Data.LVEL,_Data.BATTERY,_Data.SONARCOUNT,_Data.SONAR1,_Data.SONAR2))
                if (currentinput == _Config.CONSOLE_CLOSE):
                    SerialConnection.close()
                    thread.interrupt_main()
                if (currentinput == _Config.CONSOLE_TEST):
                    _Robot.Test()
                else:
                    print(_Config.CONSOLE_ERROR)    

if __name__ == "__main__":
    SerialConnection = serial.Serial(_Config.SERIAL_PORT,baudrate=_Config.SERIAL_BAUDRATE, timeout=_Config.SERIAL_TIMEOUT,write_timeout=0)
    print(SerialConnection.name)
    
    _Robot.Init()
    
    time.sleep(_Config.WAIT)
    
    #pkg = _Tools.Package(8,255)
    #SerialConnection.write(pkg)
    #print("sent " + str(pkg))
    
    threads = []
    threads.append(Thread(target=_Robot.Heartbeat,daemon=True))
    threads.append(Thread(target=_Robot.Read,daemon=True))
    #threads.append(Thread(target=_Robot.Write,daemon=True))
    threads.append(Thread(target=_HMI.Console,daemon=True))
    #threads.append(Thread(target=_ROS.Subscribe,daemon=True))
    #threads.append(Thread(target=_ROS.Publish,daemon=True))
    

    for thread in threads:
        thread.start()

    while _Tools.IsAnyThreadAlive(threads):
        time.sleep(0)