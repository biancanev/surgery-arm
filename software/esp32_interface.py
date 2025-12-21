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
        self.min_send_interval = 0.02
        
        print(f"ESP32 Controller initialized")
        print(f"Target: {esp32_ip}:{port}")
        self.test_connection()
    
    def test_connection(self):
        try:
            test_angles = [90.0] * 5
            packet = struct.pack('5f', *test_angles)
            self.sock.sendto(packet, (self.esp32_ip, self.port))
            self.connected = True
            print("✓ Connection test successful")
            return True
        except Exception as e:
            print(f"✗ Connection test failed: {e}")
            self.connected = False
            return False
    
    def send_joint_angles(self, joint_angles_rad):
        current_time = time.time()
        if current_time - self.last_send_time < self.min_send_interval:
            return False
        
        angles_deg = self.rad_to_servo_range(joint_angles_rad)
        
        packet = struct.pack('5f', *angles_deg)
        
        try:
            self.sock.sendto(packet, (self.esp32_ip, self.port))
            self.last_send_time = current_time
            return True
        except Exception as e:
            print(f"Send failed: {e}")
            self.connected = False
            return False
    
    def rad_to_servo_range(self, joint_angles_rad):
        angles_deg = []
        
        for i, angle_rad in enumerate(joint_angles_rad):
            deg = np.degrees(angle_rad)
            
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
        if len(angles_deg) != 5:
            print(f"Error: Expected 5 angles, got {len(angles_deg)}")
            return False
        
        packet = struct.pack('5f', *angles_deg)
        try:
            self.sock.sendto(packet, (self.esp32_ip, self.port))
            return True
        except Exception as e:
            print(f"Send failed: {e}")
            return False
    
    def close(self):
        self.sock.close()
        print("ESP32 controller closed")