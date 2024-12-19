from math import cos, sin, atan2, asin, sqrt, pi
from numpy import deg2rad, rad2deg
from geometry_msgs.msg import Quaternion


class VNYMRHandler():
    '''
    Class to handle VNYMR strings.
    '''
    def __init__(self) -> None:
        '''
        Initializes the VNYMRHandler class.
        '''
        pass

    def is_VNYMR(self, data: str) -> bool:
        '''
        Checks if the data is a VNYMR string.
        
        Args:
            data (str): The data to be checked.
            
        Returns:
            bool: True if the data is a VNYMR string, False otherwise.
        '''
        return 'VNYMR' in data
    
    def has_none(self, data: dict) -> bool:
        '''
        Checks if the data has a None value.
        
        Args:
            data (str): The data to be checked.
            
        Returns:
            bool: True if the data has a None value, False otherwise.
        '''
        if data is None:
            print("Data dictionary is None.")
            return True
        return 'None' in data.values()
    
    def euler_to_quaternion(self, yaw: float, pitch: float, roll: float) -> list:
        '''
        Converts Euler angles to a quaternion.
        
        Args:
            yaw (float): The yaw angle.
            pitch (float): The pitch angle.
            roll (float): The roll angle.
            
        Returns:
            list: The quaternion.
        '''
        # Convert degrees to radians
        yaw = deg2rad(yaw)
        pitch = deg2rad(pitch)
        roll = deg2rad(roll)

        # Calculate trigonometric values
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        # Calculate quaternion
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return [w, x, y, z]
    
    def quaternion_to_euler(self, quaternion: list, in_degrees: bool = False, normalize_q: bool = False) -> tuple[float, float, float]:
        '''
        Converts a quaternion to Euler angles.
        
        Args:
            quaternion (list): The quaternion in w,x,y,z format.
            in_degrees (bool): True to return angles in degrees, False to return angles in radians.
            normalize_q (bool): True to normalize the quaternion, False otherwise.
            
        Returns:
            list: The Euler angles.
        '''
        # Normalize quaternion
        if normalize_q:
            # Normalize quaternion
            norm = sqrt(sum(q**2 for q in quaternion))
            quaternion = [q / norm for q in quaternion]

        # Extract quaternion values
        w, x, y, z = quaternion

        # Calculate Euler angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = sqrt(1 + 2 * (w * y - x * z))
        cosp = sqrt(1 - 2 * (w * y - x * z))
        pitch = 2 * atan2(sinp, cosp) - pi / 2

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)

        if in_degrees:
            return rad2deg(yaw), rad2deg(pitch), rad2deg(roll)
        
        return yaw, pitch, roll

        
    def quaternion_to_message(self, quaternion: list) -> Quaternion:
        '''
        Converts a quaternion to a Quaternion message.
        
        Args:
            quaternion (list): The quaternion in w,x,y,z format.
            
        Returns:
            Quaternion: The Quaternion message.
        '''
        return Quaternion(w=quaternion[0], x=quaternion[1], y=quaternion[2], z=quaternion[3])
    
    def extract_data(self, data: str) -> dict:
        '''
        Extracts data from a VNYMR string.
        
        Args:
            data (str): The data to be extracted.
            
        Returns:
            dict: The extracted data.
        '''
        # Split data by comma
        vnymr_data: list = data.split(',')

        try:
            # Extract relevant data
            relevant_data: dict = {
                'yaw': float(vnymr_data[1]),
                'pitch': float(vnymr_data[2]),
                'roll': float(vnymr_data[3]),
                'mag_x': float(vnymr_data[4]) * 0.0001,
                'mag_y': float(vnymr_data[5]) * 0.0001,
                'mag_z': float(vnymr_data[6]) * 0.0001,
                'accel_x': float(vnymr_data[7]),
                'accel_y': float(vnymr_data[8]),
                'accel_z': float(vnymr_data[9]),
                'gyro_x': float(vnymr_data[10]),
                'gyro_y': float(vnymr_data[11]),
                'gyro_z': float(vnymr_data[12].split('*')[0])
            }
        except:
            print("Failed to extract data from VNYMR string.")
            return None

        # Get quaternion
        quaternion: list = self.euler_to_quaternion(relevant_data['yaw'], relevant_data['pitch'], relevant_data['roll'])
        relevant_data['quaternion'] = self.quaternion_to_message(quaternion)

        if self.has_none(relevant_data):
            return None
        
        return relevant_data