import numpy as np

class RobotArm:
    def __init__(self, dh_params):
        self.dh_params = dh_params
        self.n_joints = len(dh_params)
    
    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        transforms = [np.eye(4)]
        
        for i, (theta_offset, d, a, alpha) in enumerate(self.dh_params):
            theta_i = theta_offset + joint_angles[i]
            
            ct = np.cos(theta_i)
            st = np.sin(theta_i)
            ca = np.cos(alpha)
            sa = np.sin(alpha)
            
            T_i = np.array([
                [ct, -st*ca, st*sa, a*ct],
                [st, ct*ca, -ct*sa, a*st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])
            T = T @ T_i
            transforms.append(T.copy())
        
        return T, transforms
    
    def get_joint_positions(self, joint_angles):
        _, transforms = self.forward_kinematics(joint_angles)
        positions = [T[:3, 3] for T in transforms]
        return np.array(positions)
    
    def _compute_jacobian(self, q):
        epsilon = 1e-6
        T_current, _ = self.forward_kinematics(q)
        pos_current = T_current[:3, 3]
        
        J = np.zeros((6, self.n_joints))
        
        for i in range(self.n_joints):
            q_plus = q.copy()
            q_plus[i] += epsilon
            T_plus, _ = self.forward_kinematics(q_plus)
            pos_plus = T_plus[:3, 3]
            
            J[:3, i] = (pos_plus - pos_current) / epsilon
            
            R_plus = T_plus[:3, :3]
            R_current = T_current[:3, :3]
            R_diff = R_plus @ R_current.T
            axis_angle = self._rotation_to_axis_angle(R_diff)
            J[3:, i] = axis_angle / epsilon
        
        return J
    
    def _rotation_to_axis_angle(self, R):
        angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
        if angle < 1e-6:
            return np.zeros(3)
        axis = 1/(2*np.sin(angle)) * np.array([
            R[2,1] - R[1,2],
            R[0,2] - R[2,0],
            R[1,0] - R[0,1]
        ])
        return angle * axis
    
    def inverse_kinematics(self, target_pose, initial_guess=None, max_iter=1000, tol=1e-3, 
                          position_weight=10.0, orientation_weight=1.0):
        target_pos = target_pose[:3, 3]
        target_rot = target_pose[:3, :3]
        
        if initial_guess is None:
            q = np.zeros(self.n_joints)
        else:
            q = initial_guess.copy()
        
        best_q = q.copy()
        best_error = float('inf')
        
        for iteration in range(max_iter):
            T_current, _ = self.forward_kinematics(q)
            
            pos_current = T_current[:3, 3]
            pos_error = target_pos - pos_current
            
            R_current = T_current[:3, :3]
            R_error = target_rot @ R_current.T
            rot_error = self._rotation_to_axis_angle(R_error)
            
            error = np.concatenate([pos_error * position_weight, rot_error * orientation_weight])
            
            error_norm = np.linalg.norm(error)
            if error_norm < best_error:
                best_error = error_norm
                best_q = q.copy()
            
            pos_error_norm = np.linalg.norm(pos_error)
            rot_error_norm = np.linalg.norm(rot_error)
            
            if pos_error_norm < tol and rot_error_norm < 0.1:
                print(f"IK converged in {iteration} iterations, pos_err: {pos_error_norm:.6f}m, rot_err: {rot_error_norm:.4f}rad")
                return q, True
            
            J = self._compute_jacobian(q)
            J[:3, :] *= position_weight
            J[3:, :] *= orientation_weight
            
            damping = 0.01
            JtJ = J.T @ J + damping * np.eye(self.n_joints)
            
            try:
                dq = np.linalg.solve(JtJ, J.T @ error)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(J) @ error
            
            step_size = 0.5
            q += step_size * dq
            
            q = np.clip(q, -2*np.pi, 2*np.pi)
        
        T_final = self.forward_kinematics(best_q)[0]
        pos_final = T_final[:3, 3]
        R_final = T_final[:3, :3]
        R_error_final = target_rot @ R_final.T
        
        final_pos_error = np.linalg.norm(pos_final - target_pos)
        final_rot_error = np.linalg.norm(self._rotation_to_axis_angle(R_error_final))
        
        if final_pos_error < 0.01 and final_rot_error < 0.2:
            print(f"IK approximate solution, pos_err: {final_pos_error:.6f}m, rot_err: {final_rot_error:.4f}rad")
            return best_q, True
        
        print(f"IK failed, best pos_err: {final_pos_error:.6f}m, rot_err: {final_rot_error:.4f}rad")
        return best_q, False