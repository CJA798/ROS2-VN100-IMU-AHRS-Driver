import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
from builtin_interfaces.msg import Time
from custom_interfaces.msg import IMUmsg

from .submodules.serial_com_handler import SerialHandler
from .submodules.vnymr_handler import VNYMRHandler


class IMUDriver(Node):
    '''
    This class is used to create a ROS2 node that publishes IMU messages.
    
    Attributes:
    
    Parameters:
    '''
    def __init__(self) -> None:
        '''
        This method is used to initialize the IMUDriver class.
        '''
        # Initialize the Node class
        super().__init__('imu_publisher')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # Create a publisher
        self.imu_publisher = self.create_publisher(IMUmsg, 'imu', 10)

        # Create a serial handler
        self.get_logger().info(f"Opening serial port: {self.get_parameter('port').value} at {self.get_parameter('baudrate').value} baudrate.")
        self.sh = SerialHandler(self.get_parameter('port').value, self.get_parameter('baudrate').value)

        # Check if the serial port was successfully opened
        if self.sh.ser is None:
            self.get_logger().error("Failed to open serial port.")
            raise Exception("Failed to open serial port.")
        
        # Set output frequency
        self.set_output_frequency(40)

        # Create a VNYMR handler
        self.vnymr = VNYMRHandler()

        # Create a timer to read data from the serial port
        self.timer = self.create_timer(1/40, self.timer_callback)

    def set_output_frequency(self, frequency: int) -> None:
        '''
        Set the output frequency of the VM-100.
        
        Parameters:
            frequency (int): The desired output frequency.
        '''
        self.get_logger().info(f"Setting output frequency to {frequency} Hz.")
        self.sh.write_line(f'$VNWRG,07,{frequency}*59\r\n')
        self.get_logger().info(f"Output frequency set to {frequency} Hz.")


    def timer_callback(self) -> None:
        '''
        This method publishes an IMU message.
        '''
        # Read line from serial port
        raw_data: str = self.sh.read_line()
        self.get_logger().info(f'Read: {raw_data}')

        # Get timestamp
        timestamp: Time = self.get_clock().now().to_msg()

        # Check if data is not empty
        if raw_data is None:
            self.get_logger().warning("No data read from serial port.")
            return
        
        # Check if data is a VNYMR string
        if not self.vnymr.is_VNYMR(raw_data):
            self.get_logger().warning("String is not a VNYMR string.")
            return

        # Extract data from VNYMR string
        vnymr_data: dict = self.vnymr.extract_data(raw_data)
        #self.get_logger().info(f'Extracted: {vnymr_data}')
        
        # Check if data is valid, i.e. no missing fields
        if vnymr_data is None:
            self.get_logger().warning("Data dictionary is None.")
            return

        # Create message
        msg = IMUmsg()
        msg.header = Header(frame_id="IMU1_Frame", stamp=timestamp)

        msg.imu = Imu()
        msg.imu.header = Header(frame_id="IMU1_Frame", stamp=timestamp)
        msg.imu.orientation = vnymr_data['quaternion']
        #self.get_logger().info(f'Quaternion: {msg.imu.orientation}')
        msg.imu.angular_velocity = Vector3(x=vnymr_data['gyro_x'], y=vnymr_data['gyro_y'], z=vnymr_data['gyro_z'])
        msg.imu.linear_acceleration = Vector3(x=vnymr_data['accel_x'], y=vnymr_data['accel_y'], z=vnymr_data['accel_z'])

        msg.mag_field = MagneticField()
        msg.mag_field.header = Header(frame_id="IMU1_Frame", stamp=timestamp)
        msg.mag_field.magnetic_field = Vector3(x=vnymr_data['mag_x'], y=vnymr_data['mag_y'], z=vnymr_data['mag_z'])
        msg.mag_field.magnetic_field_covariance = [0.0] * 9

        msg.raw = raw_data
        
        # Publish message
        self.imu_publisher.publish(msg)
        #self.get_logger().info(f'Publishing: {msg}')


def main(args=None):
    # Initialize ROS2 node
    try:
        rclpy.init(args=args)

        imu_driver = IMUDriver()

        rclpy.spin(imu_driver)

    # Handle exceptions
    except Exception as e:
        print(e)
        imu_driver.sh.close_port()
        imu_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()