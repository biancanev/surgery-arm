import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons
from mpl_toolkits.mplot3d import Axes3D
from arm_model import RobotArm
from surface_model import SurgicalSurface, IncisionPlanner

class RobotSimulator:
    def __init__(self):
        self.fig = plt.figure("6-DOF Surgery Robot Simulator", figsize=(16, 9))
        self.ax = self.fig.add_subplot(111, projection='3d', position=[0.42, 0.05, 0.56, 0.92])
        
        self.dh_params = [
            [0, 0.10, 0, np.pi/2],
            [0, 0, 0.25, 0],
            [0, 0, 0.15, 0],
            [0, 0, 0, np.pi/2],
            [0, 0, 0, np.pi/2],
            [0, 0.06, 0, 0],
        ]
        
        self.robot = RobotArm(self.dh_params)

        self.home_position = np.array([
            0.0,
            np.pi/4,
            -np.pi/3,
            -np.pi/4,
            np.pi/2,
            0.0
        ])
        self.joint_angles = self.home_position.copy()

        T_initial, _ = self.robot.forward_kinematics(self.joint_angles)
        self.initial_orientation = T_initial[:3, :3]
        
        self.target_pos = np.array([0.3, 0.0, 0.3])
        self.target_orientation = self.initial_orientation.copy()
        self.wrist_roll = 0.0
        self.wrist_pitch = 0.0
        self.wrist_yaw = 0.0
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
        self.update_plot()
    
    def _setup_plot(self):
        self.ax.set_xlim([-0.5, 0.5])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([0, 0.6])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('6-DOF Surgery Robot\n(Base + Shoulder + Elbow + Roll + Pitch + Yaw)')
        
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
        slider_left = 0.05
        slider_width = 0.22
        slider_height = 0.025
        slider_spacing = 0.045
        slider_start_y = 0.94
        
        self.fig.text(slider_left, 0.97, 'JOINT ANGLES', fontsize=11, weight='bold')
        
        joint_names = ['Base', 'Shoulder', 'Elbow', 'Roll', 'Pitch', 'Yaw']
        
        self.sliders = []
        for i in range(self.robot.n_joints):
            y_pos = slider_start_y - i * slider_spacing
            ax = plt.axes([slider_left, y_pos, slider_width, slider_height])
            slider = Slider(ax, joint_names[i], -np.pi, np.pi, valinit=self.home_position[i])
            slider.on_changed(self.update_joints)
            self.sliders.append(slider)
        
        target_start_y = slider_start_y - (self.robot.n_joints + 0.5) * slider_spacing
        
        self.fig.text(slider_left, target_start_y + 0.05, 'TARGET POSITION', fontsize=11, weight='bold')
        
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
        
        self.fig.text(slider_left, orientation_start_y, 'TOOL ORIENTATION', fontsize=11, weight='bold')
        
        ax_roll = plt.axes([slider_left, orientation_start_y - 0.03, slider_width, slider_height])
        self.slider_wrist_roll = Slider(ax_roll, 'Roll', -np.pi, np.pi, valinit=0.0)
        self.slider_wrist_roll.on_changed(self.update_orientation)
        
        ax_pitch = plt.axes([slider_left, orientation_start_y - 0.03 - slider_spacing, slider_width, slider_height])
        self.slider_wrist_pitch = Slider(ax_pitch, 'Pitch', -np.pi/2, np.pi/2, valinit=0.0)
        self.slider_wrist_pitch.on_changed(self.update_orientation)
        
        ax_yaw = plt.axes([slider_left, orientation_start_y - 0.03 - 2*slider_spacing, slider_width, slider_height])
        self.slider_wrist_yaw = Slider(ax_yaw, 'Yaw', -np.pi/2, np.pi/2, valinit=0.0)
        self.slider_wrist_yaw.on_changed(self.update_orientation)
        
        surface_start_y = orientation_start_y - 3.5*slider_spacing
        
        self.fig.text(slider_left, surface_start_y, 'SURFACE CONTROLS', fontsize=11, weight='bold')
        
        ax_surf_x = plt.axes([slider_left, surface_start_y - 0.03, slider_width, slider_height])
        self.slider_surf_x = Slider(ax_surf_x, 'Surface X', -0.5, 0.5, valinit=0.35)
        
        ax_surf_y = plt.axes([slider_left, surface_start_y - 0.03 - slider_spacing, slider_width, slider_height])
        self.slider_surf_y = Slider(ax_surf_y, 'Surface Y', -0.3, 0.3, valinit=0.0)
        
        ax_surf_z = plt.axes([slider_left, surface_start_y - 0.03 - 2*slider_spacing, slider_width, slider_height])
        self.slider_surf_z = Slider(ax_surf_z, 'Surface Z', 0.0, 0.6, valinit=0.25)
        
        ax_surf_tilt = plt.axes([slider_left, surface_start_y - 0.03 - 3*slider_spacing, slider_width, slider_height])
        self.slider_surf_tilt = Slider(ax_surf_tilt, 'Tilt', -np.pi/3, np.pi/3, valinit=0.3)
        
        ax_surf_rot = plt.axes([slider_left, surface_start_y - 0.03 - 4*slider_spacing, slider_width, slider_height])
        self.slider_surf_rot = Slider(ax_surf_rot, 'Rotation', -np.pi, np.pi, valinit=0.0)
        
        control_y = surface_start_y - 5.5*slider_spacing
        
        self.fig.text(slider_left + 0.7, control_y + 0.1, 'CONTROL MODE', fontsize=11, weight='bold')
        
        ax_radio = plt.axes([slider_left + 0.7, control_y, 0.10, 0.08])
        self.radio = RadioButtons(ax_radio, ('FK', 'IK'))
        self.radio.on_clicked(self.change_mode)
        
        ax_orient_check = plt.axes([slider_left + 0.81, control_y + 0.04, 0.10, 0.04])
        self.check_orient = CheckButtons(ax_orient_check, ['Orient'], [False])
        self.check_orient.on_clicked(self.toggle_orientation_control)
        
        btn_width = 0.10
        btn_height = 0.035
        btn_y1 = control_y + 0.05
        
        ax_solve = plt.axes([slider_left + 0.7, btn_y1, btn_width, btn_height])
        self.btn_solve = Button(ax_solve, 'Solve IK', color='lightblue', hovercolor='skyblue')
        self.btn_solve.on_clicked(self.solve_ik)
        
        ax_reset = plt.axes([slider_left + 0.7 + btn_width + 0.01, btn_y1, btn_width, btn_height])
        self.btn_reset = Button(ax_reset, 'Reset', color='lightcoral', hovercolor='salmon')
        self.btn_reset.on_clicked(self.reset_robot)
        
        btn_y2 = btn_y1 - btn_height - 0.01
        
        ax_update_surface = plt.axes([slider_left + 0.7, btn_y2, btn_width, btn_height])
        self.btn_update_surface = Button(ax_update_surface, 'Update Surface', color='wheat', hovercolor='orange')
        self.btn_update_surface.on_clicked(self.update_surface_from_sliders)
        
        ax_clear = plt.axes([slider_left + 0.7 + btn_width + 0.01, btn_y2, btn_width, btn_height])
        self.btn_clear = Button(ax_clear, 'Clear Path', color='lightyellow', hovercolor='yellow')
        self.btn_clear.on_clicked(self.clear_path)
        
        btn_y3 = btn_y2 - btn_height - 0.01
        
        ax_execute = plt.axes([slider_left + 0.7, btn_y3, btn_width*2 + 0.01, btn_height])
        self.btn_execute = Button(ax_execute, 'Execute Incision', color='lightgreen', hovercolor='lime')
        self.btn_execute.on_clicked(lambda event: self.execute_surface_incision())
        
        anim_label_y = btn_y3 + 0.06
        self.fig.text(slider_left, anim_label_y, 'ANIMATION', fontsize=10, weight='bold')
        
        ax_speed = plt.axes([slider_left, anim_label_y - 0.03, slider_width, slider_height])
        self.slider_speed = Slider(ax_speed, 'Speed', 0.5, 5.0, valinit=2.0)
        
        info_y = anim_label_y - 0.08
        self.info_text = self.fig.text(slider_left, info_y, '', fontsize=9, wrap=True)
        
    def update_joints(self, val):
        if self.is_animating:
            return
        
        if self.mode == 'FK':
            for i, slider in enumerate(self.sliders):
                self.joint_angles[i] = slider.val
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
        self.wrist_yaw = self.slider_wrist_yaw.val
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
        
        Rx_yaw = np.array([
            [1, 0, 0],
            [0, np.cos(self.wrist_yaw), -np.sin(self.wrist_yaw)],
            [0, np.sin(self.wrist_yaw), np.cos(self.wrist_yaw)]
        ])
        
        self.target_orientation = Rz_roll @ Ry_pitch @ Rx_yaw
    
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
            print(f"  With orientation: roll={np.degrees(self.wrist_roll):.1f}°, pitch={np.degrees(self.wrist_pitch):.1f}°, yaw={np.degrees(self.wrist_yaw):.1f}°")
        
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
        speed = self.slider_speed.val
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
        self.joint_angles = self.home_position.copy()
        for i, slider in enumerate(self.sliders):
            slider.set_val(self.home_position[i])
        self.path_history = []
        self.info_text.set_text('Robot reset to home position')
        self.info_text.set_color('black')
        self.update_plot()
        self.fig.canvas.draw_idle()
    
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
                    info_str += f' | R: {np.degrees(self.wrist_roll):.0f}° P: {np.degrees(self.wrist_pitch):.0f}° Y: {np.degrees(self.wrist_yaw):.0f}°'
                
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
        
        print(f"Pre-computing IK for {len(self.planned_trajectory)} waypoints...")
        self.is_animating = True
        self.info_text.set_text('Pre-computing IK solutions...')
        self.info_text.set_color('blue')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        joint_waypoints = []
        current_q = self.joint_angles.copy()
        
        for i, (point, R, pose) in enumerate(self.planned_trajectory):
            q_solution, success = self.robot.inverse_kinematics(
                pose, current_q,
                position_weight=10.0, orientation_weight=1.0
            )
            
            if success:
                joint_waypoints.append(q_solution)
                current_q = q_solution
            else:
                print(f"IK failed at waypoint {i}")
                self.is_animating = False
                self.info_text.set_text(f'IK failed at waypoint {i}')
                self.info_text.set_color('red')
                self.update_plot()
                return
        
        print(f"Successfully computed {len(joint_waypoints)} IK solutions")
        print("Executing incision animation...")
        
        speed = self.slider_speed.val
        steps_per_segment = max(1, int(10 / speed))
        
        full_trajectory = []
        for i in range(len(joint_waypoints) - 1):
            segment = self.generate_trajectory(
                joint_waypoints[i], 
                joint_waypoints[i+1], 
                steps_per_segment
            )
            full_trajectory.extend(segment[:-1])
        full_trajectory.append(joint_waypoints[-1])
        
        self.path_history = []
        self.info_text.set_text('Executing incision...')
        self.info_text.set_color('blue')
        
        for step, q in enumerate(full_trajectory):
            self.joint_angles = q
            positions = self.robot.get_joint_positions(self.joint_angles)
            end_pos = positions[-1]
            self.path_history.append(end_pos.copy())
            
            self.update_plot()
            self.fig.canvas.flush_events()
            plt.pause(0.01)
        
        for i, slider in enumerate(self.sliders):
            slider.set_val(self.joint_angles[i])
        
        self.is_animating = False
        self.info_text.set_text('Incision complete!')
        self.info_text.set_color('green')
        self.update_plot()
        print("Incision complete!")

    def update_surface_from_sliders(self, event=None):
        center = np.array([
            self.slider_surf_x.val,
            self.slider_surf_y.val,
            self.slider_surf_z.val
        ])
        
        tilt = self.slider_surf_tilt.val
        rotation = self.slider_surf_rot.val
        
        normal = np.array([
            np.sin(tilt) * np.cos(rotation),
            np.sin(tilt) * np.sin(rotation),
            np.cos(tilt)
        ])
        
        print(f"\nUpdating surface:")
        print(f"  Center: {center}")
        print(f"  Normal: {normal}")
        print(f"  Tilt: {np.degrees(tilt):.1f}°, Rotation: {np.degrees(rotation):.1f}°")
        
        self.add_surface('plane', 
                        center=center,
                        normal=normal,
                        size=0.15)
        
        self.plan_surface_incision(
            start_uv=(0.3, 0.4),
            end_uv=(0.7, 0.6)
        )
        
        self.info_text.set_text('Surface updated!')
        self.info_text.set_color('green')

    def update_surface_on_startup(self):
        self.update_surface_from_sliders()

if __name__ == '__main__':
    print("="*60)
    print("6-DOF SURGERY ROBOT SIMULATION")
    print("="*60)
    print("Joints: Base (360°) + Shoulder + Elbow + Roll + Pitch + Yaw")
    print("DH Parameters:")
    print("  L1 (shoulder-elbow): 25cm")
    print("  L2 (elbow-wrist):    15cm")
    print("  L3 (wrist-tool):     6cm")
    print()
    print("Home Position:")
    print("  Base:     0° (centered)")
    print("  Shoulder: 45° (raised)")
    print("  Elbow:    -60° (bent)")
    print("  Roll:     0°")
    print("  Pitch:    0°")
    print("  Yaw:      0°")
    print("="*60)
    
    sim = RobotSimulator()

    sim.add_surface('plane', 
                    center=np.array([0.35, 0.0, 0.0]),
                    normal=np.array([0, 0.3, 1]),
                    size=0.15)
    
    sim.plan_surface_incision(
        start_uv=(0.3, 0.4),
        end_uv=(0.7, 0.6)
    )

    sim.update_surface_on_startup()
    
    sim.show()