import struct
import numpy as np

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

class PacketValidator:
    def __init__(self):
        self.passed = []
        self.failed = []
    
    def run_test(self, test_name, test_func):
        print(f"\n{Colors.BOLD}{Colors.BLUE}{test_name}{Colors.RESET}")
        print("-" * 60)
        
        try:
            test_func()
            print(f"{Colors.GREEN}✓ PASS{Colors.RESET}\n")
            self.passed.append(test_name)
            return True
        except AssertionError as e:
            print(f"{Colors.RED}✗ FAIL{Colors.RESET}")
            print(f"{Colors.RED}  Reason: {str(e)}{Colors.RESET}\n")
            self.failed.append((test_name, str(e)))
            return False
        except Exception as e:
            print(f"{Colors.RED}✗ ERROR{Colors.RESET}")
            print(f"{Colors.RED}  Error: {str(e)}{Colors.RESET}\n")
            self.failed.append((test_name, f"ERROR: {str(e)}"))
            return False
    
    def test_basic_packet(self):
        angles = [45.0, 90.0, 135.0, 60.0, 120.0]
        packet = struct.pack('5f', *angles)
        
        print(f"Input angles: {angles}")
        print(f"Packet size: {len(packet)} bytes")
        print(f"Packet (hex): {packet.hex()}")
        
        unpacked = struct.unpack('5f', packet)
        print(f"Unpacked: {list(unpacked)}")
        
        for i in range(5):
            assert abs(unpacked[i] - angles[i]) < 0.001, f"Mismatch at index {i}"
    
    def test_edge_cases(self):
        edge_cases = [
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [180.0, 180.0, 180.0, 180.0, 180.0],
            [0.0, 45.0, 90.0, 135.0, 180.0],
            [0.1, 0.2, 0.3, 0.4, 0.5],
        ]
        
        for angles in edge_cases:
            packet = struct.pack('5f', *angles)
            unpacked = struct.unpack('5f', packet)
            
            for i in range(5):
                assert abs(unpacked[i] - angles[i]) < 0.001, f"Mismatch for {angles}"
            
            print(f"✓ {angles} → {list(unpacked)}")
    
    def test_radian_conversion(self):
        test_rads = [0, np.pi/4, np.pi/2, -np.pi/4, -np.pi/2]
        expected_degs = [90, 135, 180, 45, 0]
        
        for rad, expected in zip(test_rads, expected_degs):
            deg = np.degrees(rad) + 90
            deg = np.clip(deg, 0, 180)
            
            print(f"{rad:7.4f} rad → {deg:6.1f}° (expected {expected}°)")
            assert abs(deg - expected) < 1.0, f"Conversion failed for {rad}"
    
    def test_command_packets(self):
        emergency_stop = struct.pack('B', 0xFF)
        home_cmd = struct.pack('B', 0x00)
        status_req = struct.pack('B', 0xAA)
        
        print(f"Emergency stop: {emergency_stop.hex()} ({len(emergency_stop)} byte)")
        print(f"Home command:   {home_cmd.hex()} ({len(home_cmd)} byte)")
        print(f"Status request: {status_req.hex()} ({len(status_req)} byte)")
        
        assert len(emergency_stop) == 1
        assert len(home_cmd) == 1
        assert len(status_req) == 1
    
    def print_summary(self):
        print("\n" + "=" * 70)
        print(f"{Colors.BOLD}{Colors.CYAN}PACKET VALIDATION SUMMARY{Colors.RESET}")
        print("=" * 70)
        
        total = len(self.passed) + len(self.failed)
        
        print(f"\n{Colors.BOLD}Passed Tests:{Colors.RESET}")
        for test in self.passed:
            print(f"  {Colors.GREEN}✓{Colors.RESET} {test}")
        
        if self.failed:
            print(f"\n{Colors.BOLD}Failed Tests:{Colors.RESET}")
            for test, reason in self.failed:
                print(f"  {Colors.RED}✗{Colors.RESET} {test}")
                print(f"    {Colors.RED}{reason}{Colors.RESET}")
        
        print("\n" + "=" * 70)
        print(f"Total Tests: {total}")
        print(f"{Colors.GREEN}Passed: {len(self.passed)}{Colors.RESET}")
        print(f"{Colors.RED}Failed: {len(self.failed)}{Colors.RESET}")
        print("=" * 70)
        
        if len(self.failed) == 0:
            print(f"\n{Colors.GREEN}{Colors.BOLD}ALL VALIDATION TESTS PASSED!{Colors.RESET}\n")
        else:
            print(f"\n{Colors.RED}{Colors.BOLD}SOME TESTS FAILED{Colors.RESET}\n")

def main():
    print("\n" + "=" * 70)
    print(f"{Colors.BOLD}{Colors.BLUE}PACKET FORMAT VALIDATION{Colors.RESET}")
    print("=" * 70 + "\n")
    
    validator = PacketValidator()
    
    validator.run_test("Test 1: Basic 5-float packet", validator.test_basic_packet)
    validator.run_test("Test 2: Edge cases", validator.test_edge_cases)
    validator.run_test("Test 3: Radian to degree conversion", validator.test_radian_conversion)
    validator.run_test("Test 4: Command packets", validator.test_command_packets)
    
    validator.print_summary()
    
    return 0 if len(validator.failed) == 0 else 1

if __name__ == "__main__":
    import sys
    sys.exit(main())