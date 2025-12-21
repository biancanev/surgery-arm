import socket
import struct
import threading
import time
import sys

class MockESP32Server:
    def __init__(self, ip="127.0.0.1", port=8888):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        
        self.running = False
        self.thread = None
        
        self.current_angles = [90.0] * 5
        self.packet_count = 0
        self.invalid_packets = 0
        self.emergency_stops = 0
        self.home_commands = 0
        
        self.last_packet_time = 0
        self.safety_timeout = 1.0
        
        print(f"Mock ESP32 Server started on {ip}:{port}")
        print("Simulating 5-DOF surgical robot controller")
        print("=" * 50)
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._server_loop, daemon=True)
        self.thread.start()
        print("✓ Server thread started")
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        self.sock.close()
        print("\n✓ Server stopped")
    
    def _server_loop(self):
        self.sock.settimeout(0.1)
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                self.last_packet_time = time.time()
                self._process_packet(data, addr)
                
            except socket.timeout:
                pass
            
            self._check_safety_timeout()
            time.sleep(0.001)
    
    def _process_packet(self, data, addr):
        packet_len = len(data)
        
        if packet_len == 20:
            try:
                angles = struct.unpack('5f', data)
                
                valid = True
                for i, angle in enumerate(angles):
                    if angle < -10 or angle > 190:
                        valid = False
                        print(f"✗ Invalid angle[{i}]: {angle:.1f}°")
                        self.invalid_packets += 1
                        return
                    
                    if angle != angle:
                        valid = False
                        print(f"✗ NaN detected in angle[{i}]")
                        self.invalid_packets += 1
                        return
                
                if valid:
                    self.current_angles = list(angles)
                    self.packet_count += 1
                    
                    print(f"RX [{self.packet_count:4d}]: ", end="")
                    for angle in angles:
                        print(f"{angle:6.1f}° ", end="")
                    print(f" from {addr[0]}:{addr[1]}")
                    
            except struct.error as e:
                print(f"✗ Struct unpack error: {e}")
                self.invalid_packets += 1
                
        elif packet_len == 1:
            cmd = data[0]
            
            if cmd == 0xFF:
                print(f"🛑 EMERGENCY STOP from {addr[0]}:{addr[1]}")
                self.emergency_stops += 1
                self.current_angles = [90.0] * 5
                
            elif cmd == 0x00:
                print(f"🏠 HOME command from {addr[0]}:{addr[1]}")
                self.home_commands += 1
                self.current_angles = [90.0] * 5
                
            elif cmd == 0xAA:
                print(f"📊 STATUS request from {addr[0]}:{addr[1]}")
                self._send_status(addr)
                
        else:
            print(f"✗ Invalid packet size: {packet_len} bytes (expected 20 or 1)")
            self.invalid_packets += 1
    
    def _send_status(self, addr):
        status = struct.pack('5f', *self.current_angles)
        status += struct.pack('B', 0)
        status += struct.pack('i', -50)
        status += struct.pack('L', int(time.time() * 1000))
        
        self.sock.sendto(status, addr)
    
    def _check_safety_timeout(self):
        if self.last_packet_time > 0:
            elapsed = time.time() - self.last_packet_time
            if elapsed > self.safety_timeout:
                if any(angle != 90.0 for angle in self.current_angles):
                    print(f"\n⚠ SAFETY TIMEOUT - Returning to home")
                    self.current_angles = [90.0] * 5
                self.last_packet_time = 0
    
    def print_stats(self):
        print("\n" + "=" * 50)
        print("MOCK ESP32 SERVER STATISTICS")
        print("=" * 50)
        print(f"Total packets received: {self.packet_count}")
        print(f"Invalid packets: {self.invalid_packets}")
        print(f"Emergency stops: {self.emergency_stops}")
        print(f"Home commands: {self.home_commands}")
        print(f"Current angles: {[f'{a:.1f}°' for a in self.current_angles]}")
        print("=" * 50)

if __name__ == "__main__":
    server = MockESP32Server()
    server.start()
    
    try:
        print("\nMock server running. Press Ctrl+C to stop.\n")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        server.print_stats()
        server.stop()