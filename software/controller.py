class SurgeryRobotController:
    def __init__(self, robot_arm, line_detector, camera_cal):
        self.robot = robot_arm
        self.detector = line_detector
        self.camera = camera_cal
    
    def execute_vision_guided_cut(self):
        frame = self.detector.cam.read()[1]
        lines = self.detector.detect_line(frame)
        pixel_waypoints = self.detector.lines_to_waypoints(lines)
        
        robot_waypoints = [
            self.camera.pixel_to_robot(pw) for pw in pixel_waypoints
        ]
        
        trajectory = self.robot.generate_trajectory(robot_waypoints)
        
        for pose in trajectory:
            joint_angles, success = self.robot.inverse_kinematics(pose)
            if success:
                self.send_to_motors(joint_angles)
    
    def send_to_motors(self, joint_angles):
        pass