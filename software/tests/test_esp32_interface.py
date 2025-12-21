import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import unittest
import numpy as np
import time
from esp32_interface import ESP32Controller
from mock_esp32_server import MockESP32Server

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

class ColoredTestResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.test_results = []
    
    def addSuccess(self, test):
        super().addSuccess(test)
        test_name = test._testMethodName
        print(f"{Colors.GREEN}✓ PASS{Colors.RESET} - {test_name}")
        self.test_results.append((test_name, 'PASS', None))
    
    def addError(self, test, err):
        super().addError(test, err)
        test_name = test._testMethodName
        print(f"{Colors.RED}✗ ERROR{Colors.RESET} - {test_name}")
        self.test_results.append((test_name, 'ERROR', err))
    
    def addFailure(self, test, err):
        super().addFailure(test, err)
        test_name = test._testMethodName
        print(f"{Colors.RED}✗ FAIL{Colors.RESET} - {test_name}")
        self.test_results.append((test_name, 'FAIL', err))
    
    def addSkip(self, test, reason):
        super().addSkip(test, reason)
        test_name = test._testMethodName
        print(f"{Colors.YELLOW}⊘ SKIP{Colors.RESET} - {test_name}: {reason}")
        self.test_results.append((test_name, 'SKIP', reason))

class ColoredTestRunner(unittest.TextTestRunner):
    resultclass = ColoredTestResult
    
    def run(self, test):
        result = super().run(test)
        self._print_summary(result)
        return result
    
    def _print_summary(self, result):
        print("\n" + "=" * 70)
        print(f"{Colors.BOLD}{Colors.CYAN}TEST SUMMARY{Colors.RESET}")
        print("=" * 70)
        
        passed = result.testsRun - len(result.failures) - len(result.errors)
        
        for test_name, status, err in result.test_results:
            if status == 'PASS':
                print(f"{Colors.GREEN}✓ PASS{Colors.RESET}  - {test_name}")
            elif status == 'FAIL':
                print(f"{Colors.RED}✗ FAIL{Colors.RESET}  - {test_name}")
            elif status == 'ERROR':
                print(f"{Colors.RED}✗ ERROR{Colors.RESET} - {test_name}")
            elif status == 'SKIP':
                print(f"{Colors.YELLOW}⊘ SKIP{Colors.RESET}  - {test_name}")
        
        print("=" * 70)
        print(f"Total Tests: {result.testsRun}")
        print(f"{Colors.GREEN}Passed: {passed}{Colors.RESET}")
        print(f"{Colors.RED}Failed: {len(result.failures)}{Colors.RESET}")
        print(f"{Colors.RED}Errors: {len(result.errors)}{Colors.RESET}")
        print(f"{Colors.YELLOW}Skipped: {len(result.skipped)}{Colors.RESET}")
        print("=" * 70)
        
        if result.wasSuccessful():
            print(f"\n{Colors.GREEN}{Colors.BOLD}ALL TESTS PASSED!{Colors.RESET}\n")
        else:
            print(f"\n{Colors.RED}{Colors.BOLD}SOME TESTS FAILED{Colors.RESET}\n")

class TestESP32Interface(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print(f"\n{Colors.CYAN}{Colors.BOLD}Starting Mock ESP32 Server...{Colors.RESET}")
        cls.server = MockESP32Server(ip="127.0.0.1", port=8888)
        cls.server.start()
        time.sleep(0.5)
        print(f"{Colors.GREEN}✓ Server ready{Colors.RESET}\n")
    
    @classmethod
    def tearDownClass(cls):
        print(f"\n{Colors.CYAN}Stopping server...{Colors.RESET}")
        cls.server.print_stats()
        cls.server.stop()
    
    def setUp(self):
        self.controller = ESP32Controller("127.0.0.1", port=8888, timeout=1.0)
        time.sleep(0.1)
    
    def test_01_connection(self):
        self.assertTrue(self.controller.connected)
    
    def test_02_send_zero_angles(self):
        joint_angles = np.zeros(5)
        result = self.controller.send_joint_angles(joint_angles)
        time.sleep(0.1)
        
        self.assertTrue(result)
        self.assertEqual(self.server.current_angles, [90.0] * 5)
    
    def test_03_send_positive_angles(self):
        joint_angles = np.array([np.pi/4, np.pi/3, np.pi/2, np.pi/6, 0])
        result = self.controller.send_joint_angles(joint_angles)
        time.sleep(0.1)
        
        self.assertTrue(result)
        expected = [135.0, 150.0, 180.0, 120.0, 90.0]
        
        for i in range(5):
            self.assertAlmostEqual(self.server.current_angles[i], expected[i], places=1)
    
    def test_04_send_negative_angles(self):
        joint_angles = np.array([-np.pi/4, -np.pi/6, 0, 0, -np.pi/3])
        result = self.controller.send_joint_angles(joint_angles)
        time.sleep(0.1)
        
        self.assertTrue(result)
        expected = [45.0, 60.0, 90.0, 90.0, 30.0]
        
        for i in range(5):
            self.assertAlmostEqual(self.server.current_angles[i], expected[i], places=1)
    
    def test_05_angle_clamping(self):
        joint_angles = np.array([np.pi*2, -np.pi*2, 0, 0, 0])
        result = self.controller.send_joint_angles(joint_angles)
        time.sleep(0.1)
        
        self.assertTrue(result)
        
        for angle in self.server.current_angles:
            self.assertGreaterEqual(angle, 0)
            self.assertLessEqual(angle, 180)
    
    def test_06_emergency_stop(self):
        joint_angles = np.array([np.pi/4] * 5)
        self.controller.send_joint_angles(joint_angles)
        time.sleep(0.1)
        
        initial_count = self.server.emergency_stops
        result = self.controller.emergency_stop()
        time.sleep(0.1)
        
        self.assertTrue(result)
        self.assertEqual(self.server.emergency_stops, initial_count + 1)
        self.assertEqual(self.server.current_angles, [90.0] * 5)
    
    def test_07_home_position(self):
        joint_angles = np.array([np.pi/4] * 5)
        self.controller.send_joint_angles(joint_angles)
        time.sleep(0.1)
        
        initial_count = self.server.home_commands
        result = self.controller.home_position()
        time.sleep(0.1)
        
        self.assertTrue(result)
        self.assertEqual(self.server.home_commands, initial_count + 1)
        self.assertEqual(self.server.current_angles, [90.0] * 5)
    
    def test_08_multiple_rapid_sends(self):
        for i in range(10):
            angle = np.pi/4 * (i % 4)
            joint_angles = np.array([angle] * 5)
            result = self.controller.send_joint_angles(joint_angles)
            time.sleep(0.025)
        
        self.assertGreater(self.server.packet_count, 5)
    
    def test_09_packet_format(self):
        import struct
        
        angles_deg = [45.5, 90.0, 135.7, 60.3, 120.1]
        packet = struct.pack('5f', *angles_deg)
        
        self.assertEqual(len(packet), 20)
        
        unpacked = struct.unpack('5f', packet)
        for i in range(5):
            self.assertAlmostEqual(unpacked[i], angles_deg[i], places=5)

class TestRadianToDegreeConversion(unittest.TestCase):
    def test_01_zero_conversion(self):
        controller = ESP32Controller("127.0.0.1", port=8888)
        angles_rad = np.zeros(5)
        angles_deg = controller.rad_to_servo_range(angles_rad)
        
        for angle in angles_deg:
            self.assertAlmostEqual(angle, 90.0, places=1)
    
    def test_02_pi_half_conversion(self):
        controller = ESP32Controller("127.0.0.1", port=8888)
        angles_rad = np.array([np.pi/2] * 5)
        angles_deg = controller.rad_to_servo_range(angles_rad)
        
        for angle in angles_deg:
            self.assertAlmostEqual(angle, 180.0, places=1)
    
    def test_03_negative_pi_half_conversion(self):
        controller = ESP32Controller("127.0.0.1", port=8888)
        angles_rad = np.array([-np.pi/2] * 5)
        angles_deg = controller.rad_to_servo_range(angles_rad)
        
        for angle in angles_deg:
            self.assertAlmostEqual(angle, 0.0, places=1)
    
    def test_04_conversion_formula(self):
        controller = ESP32Controller("127.0.0.1", port=8888)
        
        test_cases = [
            (0, 90),
            (np.pi/4, 135),
            (-np.pi/4, 45),
            (np.pi/2, 180),
            (-np.pi/2, 0),
            (np.pi/6, 120),
            (-np.pi/3, 30),
        ]
        
        for rad, expected_deg in test_cases:
            angles_rad = np.array([rad] * 5)
            angles_deg = controller.rad_to_servo_range(angles_rad)
            
            for angle in angles_deg:
                self.assertAlmostEqual(angle, expected_deg, places=0)

if __name__ == '__main__':
    print("\n" + "=" * 70)
    print(f"{Colors.BOLD}{Colors.BLUE}ESP32 INTERFACE UNIT TESTS{Colors.RESET}")
    print("=" * 70 + "\n")
    
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestESP32Interface))
    suite.addTests(loader.loadTestsFromTestCase(TestRadianToDegreeConversion))
    
    runner = ColoredTestRunner(verbosity=1)
    result = runner.run(suite)
    
    sys.exit(0 if result.wasSuccessful() else 1)