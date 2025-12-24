# File: software/simulator.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons
from mpl_toolkits.mplot3d import Axes3D
from arm_model import RobotArm
from surface_model import SurgicalSurface, IncisionPlanner

class RobotSimulator:
    def __init__(self, use_esp32=False, esp32_ip=None):
        self.fig = plt.figure("6-DOF Surgery Robot Simulator", figsize=(16, 9))
        self.ax = self.fig.add_subplot(111, projection='3d', position=[0.45, 0.1, 0.5, 0.85])
        
        self.dh_params = [
            [0, 0.10, 0, np.pi/2],
            [0, 0, 0.25, 0],
            [0, 0, 0.15, 0],
            [0, 0, 0, np.pi/2],
            [0, 0.03, 0, np.pi/2],
            [0, 0.03, 0, 0],
        ]
        
        self.robot = RobotArm(self.dh_params)
        self.joint_angles = np.zeros(self.robot.n_joints)
        
        self.use_esp32 = use_esp32
        self.esp32 = None
        
        if use_esp32:
            if esp32_ip is None:
                esp32_ip = input("Enter ESP32 IP address: ")
            try:
                from esp32_interface import ESP32Controller
                self.esp32 = ESP32Controller(esp32_ip)
                print("✓ ESP32 connected and ready")
            except Exception as e:
                print(f"✗ Failed to connect to ESP32: {e}")
                self.use_esp32 = False
        
        T_initial, _ = self.robot.forward_kinematics(self.joint_angles)
        self.initial_orientation = T_initial[:3, :3]
        
        self.target_pos = np.array([0.3, 0.0, 0.3])
        self.target_orientation = self.initial_orientation.copy()
        self.wrist_roll = 0.0
        self.wrist_pitch = 0.0
        self.mode = 'FK'
        self.orientation_control = False
        
        self.is_animating = False
        
        self.surfaces = []
        self.current_surface = None
        self.incision_start = None
        self.incision_end = None
        self.planned_trajectory = None
        
        self._setup_plot()
        self._setup_controls()
        
        if self.use_esp32 and self.esp32:
            self.esp32.send_joint_angles(self.joint_angles)
        
        self.update_plot()
    
    def _setup_plot(self):
        self.ax.set_xlim([-0.5, 0.5])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([0, 0.6])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        
        title = '6-DOF Surgery Robot'
        if self.use_esp32:
            title += ' [HARDWARE MODE]'
        self.ax.set_title(title)
        
        self.arm_line, = self.ax.plot([], [], [], 'o-', linewidth=3, markersize=8, color='blue')
        self.end_effector, = self.ax.plot([], [], [], 'o', markersize=12, color='red')
        self.target_marker, = self.ax.plot([], [], [], '*', markersize=20, color='green')
        self.path_line, = self.ax.plot([], [], [], '--', linewidth=2, color='orange', alpha=0.6)
        
        self.tool_x_axis, = self.ax.plot([], [], [], '-', linewidth=2, color='red', alpha=0.8)
        self.tool_y_axis, = self.ax.plot([], [], [], '-', linewidth=2, color='green', alpha=0.8)
        self.tool_z_axis, = self.ax.plot([], [], [], '-', linewidth=2, color='blue', alpha=0.8)
        
        self.target_x_axis, = self.ax.plot([], [], [], '--', linewidth=2, color='red', alpha=0.5)
        self.target_y_axis, = self.ax.plot([], [], [], '--', linewidth=2, color='green', alpha=0.5)
        self.target_z_axis, = self.ax.plot([], [], [], '--', linewidth=2, color='blue', alpha=0.5)
        
        self.path_history = []
        self.surface_plot = None
        self.incision_line = None
    
    def _setup_controls(self):
        slider_left = 0.08
        slider_width = 0.25
        slider_height = 0.025
        slider_spacing = 0.04
        slider_start_y = 0.88
        
        control_text = 'JOINT ANGLES'
        if self.use_esp32:
            control_text += ' [→ HARDWARE]'
        self.fig.text(slider_left, 0.94, control_text, fontsize=12, weight='bold')
        
        joint_names = ['Base (Stepper)', 'Shoulder', 'Elbow', 'Roll', 'Pitch', 'Yaw']
        
        self.sliders = []
        for i in range(self.robot.n_joints):
            y_pos = slider_start_y - i * slider_spacing
            ax = plt.axes([slider_left, y_pos, slider_width, slider_height])
            slider = Slider(ax, joint_names[i], -np.pi, np.pi, valinit=0)
            slider.on_changed(self.update_joints)
            self.sliders.append(slider)
        
        target_start_y = slider_start_y - (self.robot.n_joints + 0.5) * slider_spacing
        
        self.fig.text(slider_left, target_start_y + 0.05, 'TARGET POSITION', fontsize=12, weight='bold')
        
        ax_x = plt.axes([slider_left, target_start_y, slider_width, slider_height])
        self.slider_x = Slider(ax_x, 'Target X', -0.5, 0.5, valinit=0.3)
        self.slider_x.on_changed(self.update_target)
        
        ax_y = plt.axes([slider_left, target_start_y - slider_spacing, slider_width, slider_height])
        self.slider_y = Slider(ax_y, 'Target Y', -0.3, 0.3, valinit=0.0)
        self.slider_y.on_changed(self.update_target)
        
        ax_z = plt.axes([slider_left, target_start_y - 2*slider_spacing, slider_width, slider_height])
        self.slider_z = Slider(ax_z, 'Target Z', 0, 0.6, valinit=0.3)
        self.slider_z.on_changed(self.update_target)
        
        orientation_start_y = target_start_y - 3*slider_spacing - 0.05
        
        self.fig.text(slider_left, orientation_start_y + 0.05, 'TOOL ORIENTATION', fontsize=12, weight='bold')
        
        ax_roll = plt.axes([slider_left, orientation_start_y - 0.03, slider_width, slider_height])
        self.slider_wrist_roll = Slider(ax_roll, 'Roll', -np.pi, np.pi, valinit=0.0)
        self.slider_wrist_roll.on_changed(self.update_orientation)
        
        ax_pitch = plt.axes([slider_left, orientation_start_y - 0.03 - slider_spacing, slider_width, slider_height])
        self.slider_wrist_pitch = Slider(ax_pitch, 'Pitch', -np.pi/2, np.pi/2, valinit=0.0)
        self.slider_wrist_pitch.on_changed(self.update_orientation)
        
        control_y = orientation_start_y - 0.16
        
        self.fig.text(slider_left, control_y + 0.1, 'CONTROL MODE', fontsize=12, weight='bold')
        
        ax_radio = plt.axes([slider_left, control_y, 0.12, 0.08])
        self.radio = RadioButtons(ax_radio, ('FK', 'IK'))
        self.radio.on_clicked(self.change_mode)
        
        ax_orient_check = plt.axes([slider_left + 0.14, control_y + 0.04, 0.12, 0.04])
        self.check_orient = CheckButtons(ax_orient_check, ['Orient Control'], [False])
        self.check_orient.on_clicked(self.toggle_orientation_control)
        
        ax_solve = plt.axes([slider_left + 0.14, control_y, 0.11, 0.04])
        self.btn_solve = Button(ax_solve, 'Solve IK', color='lightblue', hovercolor='skyblue')
        self.btn_solve.on_clicked(self.solve_ik)
        
        ax_reset = plt.axes([slider_left, control_y - 0.06, 0.11, 0.04])
        self.btn_reset = Button(ax_reset, 'Reset', color='lightcoral', hovercolor='salmon')
        self.btn_reset.on_clicked(self.reset_robot)
        
        ax_clear = plt.axes([slider_left + 0.14, control_y - 0.06, 0.11, 0.04])
        self.btn_clear = Button(ax_clear, 'Clear Path', color='lightyellow', hovercolor='yellow')
        self.btn_clear.on_clicked(self.clear_path)
        
        if self.use_esp32:
            ax_estop = plt.axes([slider_left, control_y - 0.12, 0.11, 0.04])
            self.btn_estop = Button(ax_estop, 'E-STOP', color='red', hovercolor='darkred')
            self.btn_estop.on_clicked(self.emergency_stop)
            
            ax_home = plt.axes([slider_left + 0.14, control_y - 0.12, 0.11, 0.04])
            self.btn_home = Button(ax_home, 'Home', color='lightgreen', hovercolor='green')
            self.btn_home.on_clicked(self.home_robot)
        
        anim_label_y = control_y - 0.18
        self.fig.text(slider_left, anim_label_y, 'ANIMATION', fontsize=10, weight='bold')
        
        ax_speed = plt.axes([slider_left, control_y - 0.21, slider_width, slider_height])
        self.slider_speed = Slider(ax_speed, 'Speed', 0.5, 5.0, valinit=2.0)
        
        info_y = control_y - 0.28
        self.info_text = self.fig.text(slider_left, info_y, '', fontsize=10, wrap=True)
    
    def update_joints(self, val):
        if self.is_animating:
            return
        
        if self.mode == 'FK':
            for i, slider in enumerate(self.sliders):
                self.joint_angles[i] = slider.val
            
            if self.use_esp32 and self.esp32:
                self.esp32.send_joint_angles(self.joint_angles)
            
            self.update_plot()
    
    def update_target(self, val):
        if self.is_animating:
            return
        
        self.target_pos = np.array([
            self.slider_x.val,
            self.slider_y.val,
            self.slider_z.val
        ])
        self.update_plot()
    
    def update_orientation(self, val):
        if self.is_animating:
            return
        
        self.wrist_roll = self.slider_wrist_roll.val
        self.wrist_pitch = self.slider_wrist_pitch.val
        self.update_orientation_matrix()
        self.update_plot()
    
    def update_orientation_matrix(self):
        Rz_roll = np.array([
            [np.cos(self.wrist_roll), -np.sin(self.wrist_roll), 0],
            [np.sin(self.wrist_roll), np.cos(self.wrist_roll), 0],
            [0, 0, 1]
        ])
        
        Ry_pitch = np.array([
            [np.cos(self.wrist_pitch), 0, np.sin(self.wrist_pitch)],
            [0, 1, 0],
            [-np.sin(self.wrist_pitch), 0, np.cos(self.wrist_pitch)]
        ])
        
        self.target_orientation = Rz_roll @ Ry_pitch
    
    def toggle_orientation_control(self, label):
        self.orientation_control = not self.orientation_control
        print(f"Orientation control: {'ON' if self.orientation_control else 'OFF'}")
        self.update_plot()
    
    def change_mode(self, label):
        self.mode = label
        self.update_plot()
    
    def solve_ik(self, event):
        if self.is_animating:
            return
        
        if self.mode != 'IK':
            self.mode = 'IK'
            self.radio.set_active(1)
        
        print(f"\nSolving IK for target: {self.target_pos}")
        if self.orientation_control:
            print(f"  With orientation: roll={np.degrees(self.wrist_roll):.1f}°, pitch={np.degrees(self.wrist_pitch):.1f}°")
        
        target_pose = np.eye(4)
        target_pose[:3, 3] = self.target_pos
        target_pose[:3, :3] = self.target_orientation
        
        if self.orientation_control:
            q_solution, success = self.robot.inverse_kinematics(
                target_pose, self.joint_angles,
                position_weight=10.0, orientation_weight=1.0
            )
        else:
            q_solution, success = self.robot.inverse_kinematics(
                target_pose, self.joint_angles,
                position_weight=10.0, orientation_weight=0.01
            )
        
        print(f"Solution angles (degrees): {np.degrees(q_solution)}")
        
        if success:
            self.animate_to_target(q_solution)
        else:
            self.info_text.set_text('IK Failed - Target unreachable')
            self.info_text.set_color('red')
            self.update_plot()
    
    def generate_trajectory(self, start_angles, end_angles, num_steps=50):
        trajectory = []
        for i in range(num_steps + 1):
            t = i / num_steps
            interpolated = (1 - t) * start_angles + t * end_angles
            trajectory.append(interpolated)
        return trajectory
    
    def animate_to_target(self, target_angles):
        speed = self.slider_speed.val * 10
        num_steps = int(50 / speed)
        
        trajectory = self.generate_trajectory(
            self.joint_angles.copy(), 
            target_angles, 
            num_steps
        )
        
        self.is_animating = True
        self.path_history = []
        
        self.info_text.set_text('Animating to target...')
        self.info_text.set_color('blue')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        for step, q in enumerate(trajectory):
            self.joint_angles = q
            
            if self.use_esp32 and self.esp32:
                self.esp32.send_joint_angles(self.joint_angles)
            
            positions = self.robot.get_joint_positions(self.joint_angles)
            end_pos = positions[-1]
            self.path_history.append(end_pos.copy())
            
            self.update_plot()
            self.fig.canvas.flush_events()
            
            plt.pause(0.02)
        
        for i, slider in enumerate(self.sliders):
            slider.set_val(self.joint_angles[i])
        
        self.is_animating = False
        self.info_text.set_text('Animation complete!')
        self.info_text.set_color('green')
        self.update_plot()
    
    def reset_robot(self, event):
        if self.is_animating:
            return
        
        print("Resetting robot to home position")
        self.joint_angles = np.zeros(self.robot.n_joints)
        for i, slider in enumerate(self.sliders):
            slider.set_val(0)
        
        if self.use_esp32 and self.esp32:
            self.esp32.send_joint_angles(self.joint_angles)
        
        self.path_history = []
        self.info_text.set_text('Robot reset to home position')
        self.info_text.set_color('black')
        self.update_plot()
        self.fig.canvas.draw_idle()
    
    def emergency_stop(self, event):
        if self.use_esp32 and self.esp32:
            print("🛑 EMERGENCY STOP")
            self.esp32.emergency_stop()
            self.is_animating = False
            self.info_text.set_text('⚠️ EMERGENCY STOP ACTIVATED')
            self.info_text.set_color('red')
            self.update_plot()
    
    def home_robot(self, event):
        if self.use_esp32 and self.esp32:
            print("🏠 Homing robot")
            self.esp32.home_position()
            self.joint_angles = np.zeros(6)
            for i, slider in enumerate(self.sliders):
                slider.set_val(0)
            self.info_text.set_text('Sent home command to ESP32')
            self.info_text.set_color('green')
            self.update_plot()
    
    def clear_path(self, event):
        print("Clearing path history")
        self.path_history = []
        self.update_plot()
    
    def update_plot(self):
        positions = self.robot.get_joint_positions(self.joint_angles)
        
        self.arm_line.set_data(positions[:, 0], positions[:, 1])
        self.arm_line.set_3d_properties(positions[:, 2])
        
        end_pos = positions[-1]
        self.end_effector.set_data([end_pos[0]], [end_pos[1]])
        self.end_effector.set_3d_properties([end_pos[2]])
        
        T_current, _ = self.robot.forward_kinematics(self.joint_angles)
        R_current = T_current[:3, :3]
        
        axis_length = 0.08
        x_axis = R_current[:, 0] * axis_length
        y_axis = R_current[:, 1] * axis_length
        z_axis = R_current[:, 2] * axis_length
        
        self.tool_x_axis.set_data([end_pos[0], end_pos[0] + x_axis[0]], 
                                [end_pos[1], end_pos[1] + x_axis[1]])
        self.tool_x_axis.set_3d_properties([end_pos[2], end_pos[2] + x_axis[2]])
        
        self.tool_y_axis.set_data([end_pos[0], end_pos[0] + y_axis[0]], 
                                [end_pos[1], end_pos[1] + y_axis[1]])
        self.tool_y_axis.set_3d_properties([end_pos[2], end_pos[2] + y_axis[2]])
        
        self.tool_z_axis.set_data([end_pos[0], end_pos[0] + z_axis[0]], 
                                [end_pos[1], end_pos[1] + z_axis[1]])
        self.tool_z_axis.set_3d_properties([end_pos[2], end_pos[2] + z_axis[2]])
        
        if len(self.path_history) > 1:
            path_array = np.array(self.path_history)
            self.path_line.set_data(path_array[:, 0], path_array[:, 1])
            self.path_line.set_3d_properties(path_array[:, 2])
        else:
            self.path_line.set_data([], [])
            self.path_line.set_3d_properties([])
        
        if self.surface_plot is not None:
            self.surface_plot.remove()
            self.surface_plot = None
        
        if self.current_surface is not None:
            X, Y, Z = self.current_surface.get_mesh_for_visualization(resolution=20)
            self.surface_plot = self.ax.plot_surface(
                X, Y, Z, alpha=0.3, color='tan', edgecolor='brown', linewidth=0.5
            )
        
        if self.incision_line is not None:
            for line in self.incision_line:
                line.remove()
            self.incision_line = None
        
        if self.planned_trajectory is not None and len(self.planned_trajectory) > 0:
            incision_points = np.array([p[0] for p in self.planned_trajectory])
            self.incision_line = self.ax.plot(
                incision_points[:, 0], 
                incision_points[:, 1], 
                incision_points[:, 2],
                'r-', linewidth=3, label='Planned Incision'
            )
        
        if self.mode == 'IK':
            self.target_marker.set_data([self.target_pos[0]], [self.target_pos[1]])
            self.target_marker.set_3d_properties([self.target_pos[2]])
            
            if self.orientation_control:
                target_x_axis = self.target_orientation[:, 0] * axis_length
                target_y_axis = self.target_orientation[:, 1] * axis_length
                target_z_axis = self.target_orientation[:, 2] * axis_length
                
                self.target_x_axis.set_data([self.target_pos[0], self.target_pos[0] + target_x_axis[0]], 
                                            [self.target_pos[1], self.target_pos[1] + target_x_axis[1]])
                self.target_x_axis.set_3d_properties([self.target_pos[2], self.target_pos[2] + target_x_axis[2]])
                
                self.target_y_axis.set_data([self.target_pos[0], self.target_pos[0] + target_y_axis[0]], 
                                            [self.target_pos[1], self.target_pos[1] + target_y_axis[1]])
                self.target_y_axis.set_3d_properties([self.target_pos[2], self.target_pos[2] + target_y_axis[2]])
                
                self.target_z_axis.set_data([self.target_pos[0], self.target_pos[0] + target_z_axis[0]], 
                                            [self.target_pos[1], self.target_pos[1] + target_z_axis[1]])
                self.target_z_axis.set_3d_properties([self.target_pos[2], self.target_pos[2] + target_z_axis[2]])
            else:
                self.target_x_axis.set_data([], [])
                self.target_x_axis.set_3d_properties([])
                self.target_y_axis.set_data([], [])
                self.target_y_axis.set_3d_properties([])
                self.target_z_axis.set_data([], [])
                self.target_z_axis.set_3d_properties([])
            
            distance = np.linalg.norm(end_pos - self.target_pos)
            
            if not self.is_animating:
                info_str = f'Mode: IK | Distance: {distance:.4f}m'
                if self.orientation_control:
                    info_str += f' | Roll: {np.degrees(self.wrist_roll):.0f}° Pitch: {np.degrees(self.wrist_pitch):.0f}°'
                
                if distance < 0.01:
                    self.info_text.set_color('green')
                else:
                    self.info_text.set_color('black')
                
                self.info_text.set_text(info_str)
        else:
            self.target_marker.set_data([], [])
            self.target_marker.set_3d_properties([])
            self.target_x_axis.set_data([], [])
            self.target_x_axis.set_3d_properties([])
            self.target_y_axis.set_data([], [])
            self.target_y_axis.set_3d_properties([])
            self.target_z_axis.set_data([], [])
            self.target_z_axis.set_3d_properties([])
            if not self.is_animating:
                info_str = f'Mode: FK | End Effector: ({end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f})'
                self.info_text.set_color('black')
                self.info_text.set_text(info_str)
        
        self.fig.canvas.draw_idle()
        if self.is_animating:
            self.fig.canvas.flush_events()
    
    def show(self):
        plt.ion()
        plt.show(block=True)
    
    def add_surface(self, surface_type='plane', **params):
        surface = SurgicalSurface(surface_type, **params)
        self.surfaces.append(surface)
        self.current_surface = surface
        self.update_plot()
        print(f"Added {surface_type} surface")
    
    def plan_surface_incision(self, start_uv, end_uv):
        if self.current_surface is None:
            print("No surface selected!")
            return
        
        print(f"Planning incision from {start_uv} to {end_uv}")
        self.planned_trajectory = self.current_surface.plan_incision_trajectory(
            start_uv, end_uv, num_points=30
        )
        
        print(f"Generated {len(self.planned_trajectory)} waypoints")
        self.update_plot()
    
    def execute_surface_incision(self):
        if self.planned_trajectory is None:
            print("No trajectory planned!")
            return
        
        print("Executing surface incision...")
        self.path_history = []
        
        for i, (point, R, pose) in enumerate(self.planned_trajectory):
            q_solution, success = self.robot.inverse_kinematics(
                pose, self.joint_angles,
                position_weight=10.0, orientation_weight=1.0
            )
            
            if success:
                self.joint_angles = q_solution
                self.path_history.append(point)
                
                if self.use_esp32 and self.esp32:
                    self.esp32.send_joint_angles(self.joint_angles)
                
                self.update_plot()
                plt.pause(0.05)
            else:
                print(f"IK failed at waypoint {i}")
                break
        
        print("Incision complete!")

if __name__ == '__main__':
    import sys
    
    print("="*60)
    print("6-DOF SURGERY ROBOT SIMULATION")
    print("="*60)
    print("Joints: Stepper Base + 5 Servos")
    print("  Joint 0: Base (Stepper)")
    print("  Joint 1: Shoulder")
    print("  Joint 2: Elbow")
    print("  Joint 3: Wrist Roll")
    print("  Joint 4: Wrist Pitch")
    print("  Joint 5: Wrist Yaw/Tool")
    print("="*60)
    
    use_hardware = False
    esp32_ip = None
    
    if len(sys.argv) > 1:
        if sys.argv[1] == '--esp32':
            use_hardware = True
            if len(sys.argv) > 2:
                esp32_ip = sys.argv[2]
    else:
        response = input("\nConnect to ESP32 hardware? (y/n): ").lower()
        if response == 'y':
            use_hardware = True
            esp32_ip = input("Enter ESP32 IP address: ")
    
    sim = RobotSimulator(use_esp32=use_hardware, esp32_ip=esp32_ip)
    
    sim.add_surface('plane', 
                    center=np.array([0.35, 0.0, 0.25]),
                    normal=np.array([0, 0.3, 1]),
                    size=0.15)
    
    sim.plan_surface_incision(
        start_uv=(0.3, 0.4),
        end_uv=(0.7, 0.6)
    )
    
    sim.show()