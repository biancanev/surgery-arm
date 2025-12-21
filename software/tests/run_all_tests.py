import subprocess
import sys
import time

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

def run_test(test_file, description):
    print("\n" + "╔" + "=" * 78 + "╗")
    print(f"║  {Colors.BOLD}{description:<74}{Colors.RESET}  ║")
    print("╚" + "=" * 78 + "╝\n")
    
    result = subprocess.run([sys.executable, test_file], capture_output=False)
    
    if result.returncode == 0:
        print(f"\n{Colors.GREEN}{Colors.BOLD}✓ {description} COMPLETED{Colors.RESET}\n")
        return True
    else:
        print(f"\n{Colors.YELLOW}{Colors.BOLD}⚠ {description} HAD FAILURES{Colors.RESET}\n")
        return False

def main():
    print("\n" + "█" * 80)
    print("█" + " " * 78 + "█")
    print("█" + f"{Colors.BOLD}{Colors.MAGENTA}  ESP32 SURGICAL ROBOT - COMPLETE TEST SUITE{Colors.RESET}".center(88) + "█")
    print("█" + " " * 78 + "█")
    print("█" * 80 + "\n")
    
    tests = [
        ("validate_packets.py", "Packet Format Validation"),
        ("test_esp32_interface.py", "ESP32 Interface Unit Tests"),
        ("test_integration.py", "Integration Tests (IK → ESP32)"),
    ]
    
    results = []
    
    for test_file, description in tests:
        success = run_test(test_file, description)
        results.append((description, success))
        time.sleep(0.5)
    
    print("\n" + "=" * 80)
    print(f"{Colors.BOLD}{Colors.CYAN}FINAL TEST SUMMARY - ALL SUITES{Colors.RESET}")
    print("=" * 80)
    
    for description, success in results:
        if success:
            status = f"{Colors.GREEN}✓ PASS{Colors.RESET}"
        else:
            status = f"{Colors.RED}✗ FAIL{Colors.RESET}"
        print(f"{status:20s} - {description}")
    
    print("=" * 80)
    
    all_passed = all(success for _, success in results)
    
    if all_passed:
        print(f"\n{Colors.GREEN}{Colors.BOLD}ALL TEST SUITES PASSED{Colors.RESET}\n")
        return 0
    else:
        print(f"\n{Colors.YELLOW}{Colors.BOLD}SOME TEST SUITES HAD FAILURES{Colors.RESET}\n")
        return 1

if __name__ == "__main__":
    sys.exit(main())