import numpy as np
from scipy.interpolate import RegularGridInterpolator

class SurgicalSurface:
    """
    Represents a 3D surgical surface for incision planning.
    Can be defined parametrically or from mesh/scan data.
    """
    
    def __init__(self, surface_type='plane', **params):
        self.surface_type = surface_type
        self.params = params
        self._setup_surface()
    
    def _setup_surface(self):
        """Initialize surface based on type"""
        if self.surface_type == 'plane':
            self.center = self.params.get('center', np.array([0.3, 0.0, 0.3]))
            self.normal = self.params.get('normal', np.array([0, 0, 1]))
            self.normal = self.normal / np.linalg.norm(self.normal)
            self.size = self.params.get('size', 0.2)
            
        elif self.surface_type == 'cylinder':
            self.axis_point = self.params.get('axis_point', np.array([0.3, 0.0, 0.2]))
            self.axis_dir = self.params.get('axis_dir', np.array([0, 0, 1]))
            self.axis_dir = self.axis_dir / np.linalg.norm(self.axis_dir)
            self.radius = self.params.get('radius', 0.1)
            self.height = self.params.get('height', 0.3)
            
        elif self.surface_type == 'sphere':
            self.center = self.params.get('center', np.array([0.3, 0.0, 0.3]))
            self.radius = self.params.get('radius', 0.1)
            
        elif self.surface_type == 'mesh':
            # For future 3D scan data
            self.vertices = self.params.get('vertices')
            self.faces = self.params.get('faces')
            self._build_interpolator()
    
    def _build_interpolator(self):
        """Build interpolator for mesh-based surfaces (for future use)"""
        if self.surface_type != 'mesh':
            return
        # This will be implemented when you have scan data
        pass
    
    def get_point_on_surface(self, u, v):
        """
        Get 3D point on surface given parametric coordinates (u, v)
        u, v are in [0, 1]
        """
        if self.surface_type == 'plane':
            # Create two orthogonal vectors in the plane
            if abs(self.normal[2]) < 0.9:
                v1 = np.cross(self.normal, np.array([0, 0, 1]))
            else:
                v1 = np.cross(self.normal, np.array([1, 0, 0]))
            v1 = v1 / np.linalg.norm(v1)
            v2 = np.cross(self.normal, v1)
            
            # Map u, v from [0,1] to [-size/2, size/2]
            u_scaled = (u - 0.5) * self.size
            v_scaled = (v - 0.5) * self.size
            
            point = self.center + u_scaled * v1 + v_scaled * v2
            return point
            
        elif self.surface_type == 'cylinder':
            # Parametric cylinder
            theta = u * 2 * np.pi
            z = (v - 0.5) * self.height
            
            # Create rotation matrix for cylinder axis
            axis = self.axis_dir
            if abs(axis[2]) < 0.9:
                perp1 = np.cross(axis, np.array([0, 0, 1]))
            else:
                perp1 = np.cross(axis, np.array([1, 0, 0]))
            perp1 = perp1 / np.linalg.norm(perp1)
            perp2 = np.cross(axis, perp1)
            
            # Point on cylinder
            circle_point = self.radius * (np.cos(theta) * perp1 + np.sin(theta) * perp2)
            point = self.axis_point + circle_point + z * axis
            return point
            
        elif self.surface_type == 'sphere':
            # Parametric sphere
            theta = u * 2 * np.pi
            phi = v * np.pi
            
            x = self.radius * np.sin(phi) * np.cos(theta)
            y = self.radius * np.sin(phi) * np.sin(theta)
            z = self.radius * np.cos(phi)
            
            point = self.center + np.array([x, y, z])
            return point
        
        return np.zeros(3)
    
    def get_normal_at_point(self, u, v, epsilon=1e-4):
        """
        Compute surface normal at parametric coordinates (u, v)
        """
        if self.surface_type == 'plane':
            return self.normal
        
        # Compute numerical gradient
        p = self.get_point_on_surface(u, v)
        
        # Partial derivatives
        p_u_plus = self.get_point_on_surface(u + epsilon, v)
        p_v_plus = self.get_point_on_surface(u, v + epsilon)
        
        du = (p_u_plus - p) / epsilon
        dv = (p_v_plus - p) / epsilon
        
        # Normal is cross product
        normal = np.cross(du, dv)
        normal_norm = np.linalg.norm(normal)
        
        if normal_norm > 1e-10:
            normal = normal / normal_norm
        else:
            normal = np.array([0, 0, 1])
        
        return normal
    
    def get_mesh_for_visualization(self, resolution=20):
        """
        Generate mesh for 3D visualization
        Returns: vertices (N, 3), faces (M, 3) for triangulation
        """
        u = np.linspace(0, 1, resolution)
        v = np.linspace(0, 1, resolution)
        U, V = np.meshgrid(u, v)
        
        X = np.zeros_like(U)
        Y = np.zeros_like(U)
        Z = np.zeros_like(U)
        
        for i in range(resolution):
            for j in range(resolution):
                point = self.get_point_on_surface(U[i, j], V[i, j])
                X[i, j] = point[0]
                Y[i, j] = point[1]
                Z[i, j] = point[2]
        
        return X, Y, Z
    
    def plan_incision_trajectory(self, start_uv, end_uv, num_points=50):
        """
        Plan an incision trajectory between two parametric points
        
        Args:
            start_uv: (u, v) starting parametric coordinates
            end_uv: (u, v) ending parametric coordinates
            num_points: number of waypoints
            
        Returns:
            trajectory: list of (position, orientation) tuples
        """
        trajectory = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            u = (1 - t) * start_uv[0] + t * end_uv[0]
            v = (1 - t) * start_uv[1] + t * end_uv[1]
            
            # Get point on surface
            point = self.get_point_on_surface(u, v)
            
            # Get surface normal (tool should point into surface)
            normal = self.get_normal_at_point(u, v)
            
            # Create orientation matrix
            # Z-axis points along negative normal (into tissue)
            z_axis = -normal
            
            # X-axis along incision direction (tangent to path)
            if i < num_points - 1:
                t_next = (i + 1) / (num_points - 1)
                u_next = (1 - t_next) * start_uv[0] + t_next * end_uv[0]
                v_next = (1 - t_next) * start_uv[1] + t_next * end_uv[1]
                next_point = self.get_point_on_surface(u_next, v_next)
                x_axis = next_point - point
                if np.linalg.norm(x_axis) > 1e-6:
                    x_axis = x_axis / np.linalg.norm(x_axis)
                else:
                    x_axis = np.array([1, 0, 0])
            else:
                # Use previous direction
                if len(trajectory) > 0:
                    x_axis = trajectory[-1][1][:, 0]
                else:
                    x_axis = np.array([1, 0, 0])
            
            # Make x_axis orthogonal to z_axis
            x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
            if np.linalg.norm(x_axis) > 1e-6:
                x_axis = x_axis / np.linalg.norm(x_axis)
            else:
                # If x and z are parallel, choose arbitrary orthogonal
                if abs(z_axis[0]) < 0.9:
                    x_axis = np.cross(z_axis, np.array([1, 0, 0]))
                else:
                    x_axis = np.cross(z_axis, np.array([0, 1, 0]))
                x_axis = x_axis / np.linalg.norm(x_axis)
            
            # Y-axis completes the frame
            y_axis = np.cross(z_axis, x_axis)
            
            # Build rotation matrix
            R = np.column_stack([x_axis, y_axis, z_axis])
            
            # Build full pose
            pose = np.eye(4)
            pose[:3, :3] = R
            pose[:3, 3] = point
            
            trajectory.append((point, R, pose))
        
        return trajectory


class IncisionPlanner:
    """
    Plans incision trajectories on surgical surfaces
    """
    
    def __init__(self, surface):
        self.surface = surface
    
    def plan_straight_line_incision(self, start_point, end_point, num_points=50):
        """
        Plan incision as straight line on surface (geodesic approximation)
        
        Args:
            start_point: 3D point on surface
            end_point: 3D point on surface
            num_points: number of waypoints
        """
        # Project points to parametric space (simplified - assumes we can invert)
        # For now, use simple linear interpolation in 3D and project to surface
        
        trajectory = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            # Linear interpolation in 3D
            point_3d = (1 - t) * start_point + t * end_point
            
            # Project to surface (find closest parametric coordinates)
            # For simple surfaces, use analytical projection
            # For complex surfaces, use optimization
            
            u, v = self._project_to_surface(point_3d)
            point_on_surface = self.surface.get_point_on_surface(u, v)
            normal = self.surface.get_normal_at_point(u, v)
            
            # Tool orientation (Z points into surface)
            z_axis = -normal
            
            # X-axis along incision
            if i < num_points - 1:
                next_3d = (1 - (i+1)/(num_points-1)) * start_point + ((i+1)/(num_points-1)) * end_point
                u_next, v_next = self._project_to_surface(next_3d)
                next_on_surface = self.surface.get_point_on_surface(u_next, v_next)
                x_axis = next_on_surface - point_on_surface
                if np.linalg.norm(x_axis) > 1e-6:
                    x_axis = x_axis / np.linalg.norm(x_axis)
                else:
                    x_axis = np.array([1, 0, 0])
            else:
                x_axis = trajectory[-1][1][:, 0] if trajectory else np.array([1, 0, 0])
            
            # Orthogonalize
            x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
            if np.linalg.norm(x_axis) > 1e-6:
                x_axis = x_axis / np.linalg.norm(x_axis)
            else:
                x_axis = np.array([1, 0, 0])
            
            y_axis = np.cross(z_axis, x_axis)
            
            R = np.column_stack([x_axis, y_axis, z_axis])
            
            pose = np.eye(4)
            pose[:3, :3] = R
            pose[:3, 3] = point_on_surface
            
            trajectory.append((point_on_surface, R, pose))
        
        return trajectory
    
    def _project_to_surface(self, point_3d):
        """
        Project a 3D point to the surface (find closest u, v)
        This is simplified - for complex surfaces use optimization
        """
        if self.surface.surface_type == 'plane':
            # Project to plane
            v_to_point = point_3d - self.surface.center
            
            # Get basis vectors
            normal = self.surface.normal
            if abs(normal[2]) < 0.9:
                v1 = np.cross(normal, np.array([0, 0, 1]))
            else:
                v1 = np.cross(normal, np.array([1, 0, 0]))
            v1 = v1 / np.linalg.norm(v1)
            v2 = np.cross(normal, v1)
            
            # Project onto plane
            proj = v_to_point - np.dot(v_to_point, normal) * normal
            
            # Get u, v coordinates
            u_coord = np.dot(proj, v1) / self.surface.size + 0.5
            v_coord = np.dot(proj, v2) / self.surface.size + 0.5
            
            # Clamp to [0, 1]
            u = np.clip(u_coord, 0, 1)
            v = np.clip(v_coord, 0, 1)
            
            return u, v
        
        # For other surfaces, use numerical optimization (simplified here)
        # In practice, you'd use scipy.optimize.minimize
        return 0.5, 0.5