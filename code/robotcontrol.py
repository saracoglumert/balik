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
    SONAR           = []
    COMMANDS        = []

class _Config:
    SERIAL_PORT                 = "COM5"
    SERIAL_BAUDRATE             = 9600
    SERIAL_TIMEOUT              = 0.5

    NABIZ                       = 0.1

    PACKAGE_HEADER1             = 'FA'
    PACKAGE_HEADER2             = 'FB'
    PACKAGE_ARG_TYPE_POSITIVE   = "3B"
    PACKAGE_ARG_TYPE_NEGATIVE   = "1B"
    PACKAGE_BYTE_COUNT          = "06"
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

    CONSOLE_TRANSLATE_FORWARD   = "TF"
    CONSOLE_TRANSLATE_BACKWARD  = "TB"
    CONSOLE_ROTATE_RIGHT        = "RR"
    CONSOLE_ROTATE_LEFT         = "RL"
    CONSOLE_READ                = "READ"
    CONSOLE_ERROR               = "Argument error."
    
class _Tools:
    @staticmethod
    def IsAnyThreadAlive(threads):
        return True in [t.is_alive() for t in threads]

    @staticmethod
    def Package_Command(input):
        temp = str(hex(input))
        temp = temp.replace("0x","")
        if (len(temp) == 1):
            temp = "0" + temp
        temp = temp.upper()
        return temp
    
    @staticmethod
    def Package_Arguments(input):
        temp = str(hex(input))
        temp = temp.replace("0x","")
        if (len(temp) == 1):
            temp = "0" + temp
        temp = temp.upper()
        if(len(temp) % 2 == 1):
            temp = "0" + temp
        temp2 = [temp[i:i+2] for i in range(0, len(temp), 2)]
        if (len(temp2) == 1):
            temp2.insert(0,"00")
        return temp2

    @staticmethod
    def Package_Checksum(input1,input2,input3,input4):
        temp1 = int(input1+input2,16)
        temp2 = int(input3+input4,16)
        temp = hex(temp1 + temp2)
        temp = temp.replace("0x","")
        temp = temp.upper()
        if (len(temp) % 2 == 1):
            temp = "0" + temp
        temp2 = [temp[i:i+2] for i in range(0, len(temp), 2)]
        return temp2
            
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
        temp += _Tools.Package_Checksum(temp[3],temp[4],temp[5],temp[6])

        return bytes.fromhex("".join(temp))

class _Robot:
    @staticmethod
    def Init():
        SerialConnection.write(_Config.PACKAGE_SYNC1)
        SerialConnection.write(_Config.PACKAGE_SYNC2)
        SerialConnection.write(_Config.PACKAGE_SYNC3)
        time.sleep(1)
        SerialConnection.write(_Config.PACKAGE_OPEN)
        _Data.SYNC = True
        time.sleep(1)
        SerialConnection.write(_Config.PACKAGE_SONAR_ENABLE)
        time.sleep(1)
        SerialConnection.write(_Config.PACKAGE_ENABLE)
        _Data.MOTOR_STATUS = True

    @staticmethod
    def Heartbeat():
        while True:
            SerialConnection.write(_Config.PACKAGE_PULSE)
            time.sleep(_Config.NABIZ)
    
    @staticmethod
    def Read():
        temp = b''
        while True:
            while True:
                current = SerialConnection.read(1)
                if (current == bytes.fromhex(_Config.PACKAGE_HEADER1)):
                    break
                else:
                    temp += current
            print(temp)

    @staticmethod
    def Write():
        while True:
            if (len(_Data.COMMANDS) > 0):
                for command in _Data.COMMANDS:
                    _Robot.Execute(command)

    @staticmethod
    def SetOrigin():
        SerialConnection.write(_Config.PACKAGE_SETORIGIN)

    @staticmethod
    def SetTranslationA(input):
        _Tools.Package(5,input)

    @staticmethod
    def SetTranslationV(input):
        _Tools.Package(6,input)

    @staticmethod
    def SetRotationV(input):
        _Tools.Package(10,input)

    @staticmethod
    def Translate(input):
        _Tools.Package(8,input)
        
    @staticmethod
    def Rotate(input):
        _Tools.Package(9,input)

    @staticmethod
    def Execute(input):
        if(_HMI.IsValid(input)):
            header = input[:2]
            argument = int(input[2:])
            match header:
                case _Config.CONSOLE_TRANSLATE_FORWARD:
                    _Robot.Translate(argument)
                case _Config.CONSOLE_TRANSLATE_BACKWARD:
                    _Robot.Translate(-argument)
                case _Config.CONSOLE_ROTATE_RIGHT:
                    _Robot.Rotate(-argument)
                case _Config.CONSOLE_ROTATE_LEFT:
                    _Robot.Rotate(argument)

class _HMI:
    @staticmethod
    def IsValid(input):
        header = input[:2]
        argument = input[2:]
        if (argument.isnumeric()):
            return True
        else:
            return False
    
    @staticmethod
    def Console():
        while True:
            currentinput = input(">")
            if (_HMI.IsValid(currentinput)):
                _Robot.Execute(currentinput)
            else:
                if(currentinput == _Config.CONSOLE_READ):
                    print("read")
                else:
                    print(_Config.CONSOLE_ERROR)

    @staticmethod
    def Queue(input):
        if (_HMI.IsValid(input)):
            _Data.COMMANDS.append(input)
        else:
            print(_Config.CONSOLE_ERROR)

if __name__ == "__main__":
    SerialConnection = serial.Serial(_Config.SERIAL_PORT,baudrate=_Config.SERIAL_BAUDRATE, timeout=_Config.SERIAL_TIMEOUT)
    print(SerialConnection.name)
    
    _Robot.Init()
    
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