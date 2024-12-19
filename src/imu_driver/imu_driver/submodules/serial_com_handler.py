import serial


class SerialHandler():
    '''
    A class to handle serial communication with a BU-353N USB GPS receiver.

    Attributes:
        port (str): Serial port used to communicate with the GPS receiver.
        baudrate (int): Baudrate used to communicate with the GPS receiver.
        ser (serial.Serial): Serial object used to communicate with the GPS receiver.
    '''    
    def __init__(self, port:str, baudrate:int) -> None:
        '''
        Initializes the SerialHandler class.

        Args:
            port (str): Serial port used to communicate with the GPS receiver.
            baudrate (int): Baudrate used to communicate with the GPS receiver.

        Returns:
            None

        Raises:
            serial.SerialException: If the serial port cannot be opened.
        '''
        # Initialize attributes
        self.port: str = port
        self.baudrate: int = baudrate
        self.ser: serial.Serial = None

        # Open serial port
        for _ in range(5):
            try:
                self.ser = serial.Serial(port, baudrate, timeout=1)
                break
            except serial.SerialException as e:
                print(e)

    def read_line(self) -> str:
        '''
        Reads a line from the serial port.

        Args:
            None
        
        Returns:
            str: line read from the serial port.
        
        Raises:
            None
        '''
        try:
            return self.ser.readline().decode()
        except:
            print("Failed to read line from serial port.")
            return None
        
    def write_line(self, data: str) -> None:
        '''
        Writes a line to the serial port.

        Args:
            data (str): The data to be written.

        Raises:
            None
        '''
        try:
            self.ser.write(data.encode('utf-8'))
        except:
            print("Failed to write line to serial port.")

    def close_port(self) -> None:
        '''
        Closes the serial port.
        '''
        self.ser.close()
        self.ser = None
            
