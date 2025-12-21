import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import time
from mock_esp32_server import MockESP32Server
from esp32_interface import ESP32Controller
from arm_model import RobotArm

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

class IntegrationTest:
    def __init__(self):
        self.passed = []
        self.failed = []
        self.server = None
        self.controller = None
        self.robot = None
    
    def setup(self):
        print(f"{Colors.CYAN}{Colors.BOLD}Setting up test environment...{Colors.RESET}")
        
        self.server = MockESP32Server(ip="127.0.0.1", port=8888)
        self.server.start()
        time.sleep(0.5)
        
        dh_params = [
            [0, 0.10, 0, np.pi/2],
            [0, 0, 0.25, 0],
            [0, 0, 0.15, 0],
            [0, 0, 0, np.pi/2],
            [0, 0.06, 0, 0],
        ]
        
        self.robot = RobotArm(dh_params)
        self.controller = ESP32Controller("127.0.0.1", port=8888)
        
        print(f"{Colors.GREEN}✓ Environment ready{Colors.RESET}\n")
    
    def teardown(self):
        print(f"\n{Colors.CYAN}Cleaning up...{Colors.RESET}")
        if self.server:
            self.server.print_stats()
            self.server.stop()
    
    def run_test(self, test_name, test_func):
        print(f"\n{Colors.BOLD}{Colors.BLUE}Running: {test_name}{Colors.RESET}")
        print("-" * 60)
        
        try:
            test_func()
            print(f"{Colors.GREEN}✓ PASS{Colors.RESET} - {test_name}\n")
            self.passed.append(test_name)
            return True
        except AssertionError as e:
            print(f"{Colors.RED}✗ FAIL{Colors.RESET} - {test_name}")
            print(f"{Colors.RED}  Reason: {str(e)}{Colors.RESET}\n")
            self.failed.append((test_name, str(e)))
            return False
        except Exception as e:
            print(f"{Colors.RED}✗ ERROR{Colors.RESET} - {test_name}")
            print(f"{Colors.RED}  Error: {str(e)}{Colors.RESET}\n")
            self.failed.append((test_name, f"ERROR: {str(e)}"))
            return False
    
    def test_forward_kinematics_to_esp32(self):
        test_angles = np.array([0, np.pi/4, np.pi/3, 0, 0])
        
        T, _ = self.robot.forward_kinematics(test_angles)
        print(f"FK end effector position: {T[:3, 3]}")
        
        result = self.controller.send_joint_angles(test_angles)
        time.sleep(0.2)
        
        assert result, "Failed to send joint angles"
        print(f"✓ Sent to ESP32: {np.degrees(test_angles)}")
        print(f"✓ ESP32 received: {self.server.current_angles}")
    
    def test_inverse_kinematics_to_esp32(self):
        target_pose = np.eye(4)
        target_pose[:3, 3] = np.array([0.3, 0.1, 0.15])
        
        q_solution, success = self.robot.inverse_kinematics(
            target_pose,
            initial_guess=np.zeros(5),
            position_weight=10.0,
            orientation_weight=0.01
        )
        
        assert success, "IK failed to converge"
        
        print(f"IK solution (rad): {q_solution}")
        print(f"IK solution (deg): {np.degrees(q_solution)}")
        
        result = self.controller.send_joint_angles(q_solution)
        time.sleep(0.2)
        
        assert result, "Failed to send IK solution"
        print(f"✓ ESP32 received: {self.server.current_angles}")
        
        T_verify, _ = self.robot.forward_kinematics(q_solution)
        position_error = np.linalg.norm(T_verify[:3, 3] - target_pose[:3, 3])
        print(f"✓ Position error: {position_error*1000:.2f}mm")
        
        assert position_error < 0.01, f"Position error too large: {position_error*1000:.2f}mm"
    
    def test_trajectory_execution(self):
        start_angles = np.array([0, 0, 0, 0, 0])
        end_angles = np.array([np.pi/4, np.pi/3, np.pi/4, 0, 0])
        
        num_steps = 10
        trajectory = []
        for i in range(num_steps + 1):
            t = i / num_steps
            q = (1 - t) * start_angles + t * end_angles
            trajectory.append(q)
        
        print(f"Executing {len(trajectory)} waypoints...")
        
        for i, q in enumerate(trajectory):
            result = self.controller.send_joint_angles(q)
            assert result, f"Failed at waypoint {i}"
            time.sleep(0.05)
        
        print(f"✓ Trajectory complete")
        print(f"✓ Final position: {self.server.current_angles}")
    
    def test_emergency_stop(self):
        self.controller.send_joint_angles(np.array([np.pi/6] * 5))
        time.sleep(0.1)
        print(f"Robot at: {self.server.current_angles}")
        
        self.controller.emergency_stop()
        time.sleep(0.1)
        print(f"✓ After emergency stop: {self.server.current_angles}")
        
        assert self.server.current_angles == [90.0] * 5, "Emergency stop failed to home servos"
    
    def test_home_command(self):
        self.controller.send_joint_angles(np.array([np.pi/4] * 5))
        time.sleep(0.1)
        
        self.controller.home_position()
        time.sleep(0.1)
        print(f"✓ After home command: {self.server.current_angles}")
        
        assert self.server.current_angles == [90.0] * 5, "Home command failed"
    
    def print_summary(self):
        print("\n" + "=" * 70)
        print(f"{Colors.BOLD}{Colors.CYAN}INTEGRATION TEST SUMMARY{Colors.RESET}")
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
            print(f"\n{Colors.GREEN}{Colors.BOLD}ALL INTEGRATION TESTS PASSED!{Colors.RESET}\n")
        else:
            print(f"\n{Colors.RED}{Colors.BOLD}SOME TESTS FAILED{Colors.RESET}\n")

def main():
    print("\n" + "=" * 70)
    print(f"{Colors.BOLD}{Colors.BLUE}INTEGRATION TEST: IK → ESP32{Colors.RESET}")
    print("=" * 70 + "\n")
    
    test_suite = IntegrationTest()
    test_suite.setup()
    
    test_suite.run_test("Forward Kinematics → ESP32", test_suite.test_forward_kinematics_to_esp32)
    test_suite.run_test("Inverse Kinematics → ESP32", test_suite.test_inverse_kinematics_to_esp32)
    test_suite.run_test("Trajectory Execution", test_suite.test_trajectory_execution)
    test_suite.run_test("Emergency Stop", test_suite.test_emergency_stop)
    test_suite.run_test("Home Command", test_suite.test_home_command)
    
    test_suite.teardown()
    test_suite.print_summary()
    
    sys.exit(0 if len(test_suite.failed) == 0 else 1)

if __name__ == "__main__":
    main()