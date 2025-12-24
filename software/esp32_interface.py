import socket
import struct
import time
import numpy as np

class ESP32Controller:
    def __init__(self, esp32_ip, port=8888, timeout=0.5):
        self.esp32_ip = esp32_ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.connected = False
        self.last_send_time = 0
        self.min_send_interval = 0.1
        
        print(f"ESP32 Controller initialized")
        print(f"Target: {esp32_ip}:{port}")
        self.test_connection()
    
    def test_connection(self):
        try:
            test_angles = [0.0, 90.0, 90.0, 90.0, 90.0, 90.0]
            packet = struct.pack('6f', *test_angles)
            self.sock.sendto(packet, (self.esp32_ip, self.port))
            self.connected = True
            print("✓ Connection test successful")
            return True
        except Exception as e:
            print(f"✗ Connection test failed: {e}")
            self.connected = False
            return False
    
    def send_joint_angles(self, joint_angles_rad):
        """
        Send joint angles in radians
        joint_angles_rad[0] = base (stepper, can be any angle)
        joint_angles_rad[1-4] = servos (mapped to 0-180°)
        """
        current_time = time.time()
        if current_time - self.last_send_time < self.min_send_interval:
            return False
        
        angles_deg = self.rad_to_servo_range(joint_angles_rad)
        
        packet = struct.pack('6f', *angles_deg)
        
        try:
            self.sock.sendto(packet, (self.esp32_ip, self.port))
            self.last_send_time = current_time
            return True
        except Exception as e:
            print(f"Send failed: {e}")
            self.connected = False
            return False
    
    def rad_to_servo_range(self, joint_angles_rad):
        """
        Convert robot joint angles (radians) to servo angles (degrees)
        Joint 0 (base/stepper): radians -> degrees (no offset)
        Joints 1-4 (servos): radians -> degrees with 90° offset
        """
        angles_deg = []
        
        for i, angle_rad in enumerate(joint_angles_rad):
            deg = np.degrees(angle_rad)
            
            if i == 0:
                servo_angle = deg
            else:
                servo_angle = deg + 90
                servo_angle = np.clip(servo_angle, 0, 180)
            
            angles_deg.append(servo_angle)
        
        return angles_deg
    
    def emergency_stop(self):
        print("🛑 Sending emergency stop command")
        try:
            packet = struct.pack('B', 0xFF)
            self.sock.sendto(packet, (self.esp32_ip, self.port))
            return True
        except Exception as e:
            print(f"Emergency stop failed: {e}")
            return False
    
    def home_position(self):
        print("🏠 Sending home command")
        try:
            packet = struct.pack('B', 0x00)
            self.sock.sendto(packet, (self.esp32_ip, self.port))
            return True
        except Exception as e:
            print(f"Home command failed: {e}")
            return False
    
    def send_angles_deg(self, angles_deg):
        """Send angles directly in degrees (for testing)"""
        if len(angles_deg) != 6:
            print(f"Error: Expected 6 angles, got {len(angles_deg)}")
            return False
        
        packet = struct.pack('6f', *angles_deg)
        try:
            self.sock.sendto(packet, (self.esp32_ip, self.port))
            return True
        except Exception as e:
            print(f"Send failed: {e}")
            return False
    
    def close(self):
        self.sock.close()
        print("ESP32 controller closed")