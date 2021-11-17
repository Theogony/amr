import serial, time

class MotorDriver:
    def __init__(self, port="/dev/ttyUSB1"):
        """
        Init communication, set default settings,
        """
        self.serialport = self.serialSetup(port)
        #self.serialport = serial.Serial(port)
    def set_speed(self, speed):
        """
        Set target motor speed (RPM)
        """
        string_out = "{\"cmd\":\"target_spd\",\"payload\":{\"value\":" + str(speed) + "}}\n"
        return self.query(string_out)
    def stop(self):
        """
        Stop motor
        """
        string_out = "{\"cmd\":\"run\",\"payload\":{\"value\":0}}\n"
        return self.query(string_out)          
    def start(self):
        """
        Start motor
        {"cmd":"run","payload":{"value":1}}
        """
        print("sending start command")
        string_out = "{\"cmd\":\"run\",\"payload\":{\"value\":1}}\n"
        return self.query(string_out)
    def serialSetup(self, port):
        """
        Initialize serial communication
        """
        try:
            self.serialPort = serial.Serial(port=port, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
            return(True)
        except serial.serialutil.SerialException:
            print("DEVICE NOT CONNECTED!")
            return(False)
    def query(self, dataOut):
        """
        send data and wait for reply
        """
        if self.serialport == True:
            if  self.serialPort.isOpen():
                try:
                    self.serialPort.write(str.encode(dataOut))
                    dataIn = ""
                    while(True):
                        # Wait until there is data waiting in the serial buffer
                        if(self.serialPort.in_waiting > 0):
                            while(self.serialPort.in_waiting > 0):
                            # Read data out of the buffer until a carraige return / new line is found
                                self.serialString = self.serialPort.readline()
                                dataIn += self.serialString.decode('Ascii')
                            return dataIn
                except Exception: 
                    print (" error send", dataOut)  
        else:
                print (" error send", dataOut)  
                return (False)
    def send(self, dataOut):
        """
        send data without waiting for reply
        """
        self.serialPort.write(str.encode(dataOut))
        time.sleep(0.1)


