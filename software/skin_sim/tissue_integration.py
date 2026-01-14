import numpy as np
from typing import Tuple, List, Optional
from tissue_fem import FEMTissueSimulator
from tissue_mesh import TissueMesh, create_layered_tissue_mesh
from tissue_materials import get_material_model, TISSUE_LIBRARY
from tissue_cutting import TissueCuttingSimulator

class SurgicalTissueEnvironment:
    def __init__(self, tissue_type: str = 'multilayer', position: np.ndarray = None):
        self.tissue_type = tissue_type
        self.position = position if position is not None else np.array([0.3, 0.0, 0.2])
        
        self.mesh = None
        self.fem_simulator = None
        self.cutting_simulator = None
        
        self._create_tissue()
    
    def _create_tissue(self):
        if self.tissue_type == 'multilayer':
            layer_specs = [
                {
                    'name': 'epidermis',
                    'thickness': 0.001,
                    'width': 0.15,
                    'height': 0.15,
                    'z_resolution': 2,
                    'x_offset': self.position[0] - 0.075,
                    'y_offset': self.position[1] - 0.075
                },
                {
                    'name': 'dermis',
                    'thickness': 0.002,
                    'width': 0.15,
                    'height': 0.15,
                    'z_resolution': 3,
                    'x_offset': self.position[0] - 0.075,
                    'y_offset': self.position[1] - 0.075
                },
                {
                    'name': 'subcutaneous_fat',
                    'thickness': 0.005,
                    'width': 0.15,
                    'height': 0.15,
                    'z_resolution': 4,
                    'x_offset': self.position[0] - 0.075,
                    'y_offset': self.position[1] - 0.075
                }
            ]
            
            self.mesh = create_layered_tissue_mesh(layer_specs, xy_resolution=(8, 8))
            
            material_models = [
                get_material_model('epidermis', 'neo_hookean'),
                get_material_model('dermis', 'neo_hookean'),
                get_material_model('subcutaneous_fat', 'neo_hookean')
            ]
        
        elif self.tissue_type == 'simple':
            from tissue_mesh import create_box_mesh
            self.mesh = create_box_mesh(
                dimensions=(0.15, 0.15, 0.01),
                resolution=(10, 10, 3),
                material_id=0
            )
            
            self.mesh.nodes[:, 0] += self.position[0] - 0.075
            self.mesh.nodes[:, 1] += self.position[1] - 0.075
            self.mesh.nodes[:, 2] += self.position[2]
            
            material_models = [get_material_model('dermis', 'neo_hookean')]
        
        else:
            raise ValueError(f"Unknown tissue type: {self.tissue_type}")
        
        self.fem_simulator = FEMTissueSimulator(
            mesh=self.mesh,
            material_models=material_models,
            dt=0.001,
            gravity=np.array([0, 0, -9.81])
        )
        
        bottom_nodes = []
        z_min = np.min(self.mesh.nodes[:, 2])
        for i, node in enumerate(self.mesh.nodes):
            if abs(node[2] - z_min) < 0.0001:
                bottom_nodes.append(i)
        
        self.fem_simulator.set_fixed_nodes(bottom_nodes)
        
        self.cutting_simulator = TissueCuttingSimulator(self.fem_simulator)
    
    def step(self, use_implicit: bool = False):
        if use_implicit:
            self.fem_simulator.step_implicit_euler()
        else:
            self.fem_simulator.step_explicit_euler()
        
        self.cutting_simulator.update_cut_separation()
    
    def get_surface_at_point(self, point: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        deformed_mesh = self.fem_simulator.get_deformed_mesh()
        
        min_dist = float('inf')
        nearest_point = None
        nearest_normal = None
        
        for triangle in deformed_mesh.surface_triangles:
            n0, n1, n2 = triangle
            p0 = deformed_mesh.nodes[n0]
            p1 = deformed_mesh.nodes[n1]
            p2 = deformed_mesh.nodes[n2]
            
            centroid = (p0 + p1 + p2) / 3
            
            v1 = p1 - p0
            v2 = p2 - p0
            normal = np.cross(v1, v2)
            normal = normal / np.linalg.norm(normal)
            
            dist = np.linalg.norm(point - centroid)
            if dist < min_dist:
                min_dist = dist
                nearest_point = centroid
                nearest_normal = normal
        
        return nearest_point, nearest_normal
    
    def apply_tool_contact(self, tool_position: np.ndarray, 
                          tool_orientation: np.ndarray,
                          contact_force: float = 0.0):
        deformed_mesh = self.fem_simulator.get_deformed_mesh()
        
        contact_radius = 0.005
        
        contact_nodes = []
        for i, node in enumerate(deformed_mesh.nodes):
            if np.linalg.norm(node - tool_position) < contact_radius:
                contact_nodes.append(i)
        
        if contact_nodes and contact_force > 0:
            force_per_node = contact_force * tool_orientation / len(contact_nodes)
            
            for node_idx in contact_nodes:
                self.fem_simulator.apply_point_force(node_idx, force_per_node)
    
    def apply_cutting(self, tool_position: np.ndarray,
                     tool_orientation: np.ndarray,
                     cutting_force: float) -> bool:
        blade_direction = tool_orientation / np.linalg.norm(tool_orientation)
        blade_length = 0.01
        
        cut_successful = self.cutting_simulator.apply_scalpel_cut(
            blade_position=tool_position,
            blade_direction=blade_direction,
            blade_length=blade_length,
            cutting_force=cutting_force
        )
        
        return cut_successful
    
    def get_visualization_data(self) -> dict:
        deformed_mesh = self.fem_simulator.get_deformed_mesh()
        nodes, triangles = deformed_mesh.get_surface_mesh()
        
        cut_surfaces = self.cutting_simulator.get_cut_surface_geometry()
        
        return {
            'surface_nodes': nodes,
            'surface_triangles': triangles,
            'cut_surfaces': cut_surfaces,
            'displacement_magnitude': np.linalg.norm(
                self.fem_simulator.displacements.reshape(-1, 3), axis=1
            ),
            'strain_energy': self.fem_simulator.get_strain_energy(),
            'kinetic_energy': self.fem_simulator.get_kinetic_energy()
        }

class RobotTissueInteraction:
    def __init__(self, tissue_env: SurgicalTissueEnvironment):
        self.tissue = tissue_env
        self.contact_stiffness = 1000.0
        self.contact_damping = 10.0
    
    def compute_contact_force(self, tool_tip_position: np.ndarray,
                             tool_velocity: np.ndarray) -> Tuple[np.ndarray, bool]:
        surface_point, surface_normal = self.tissue.get_surface_at_point(tool_tip_position)
        
        penetration_vector = tool_tip_position - surface_point
        penetration_depth = np.dot(penetration_vector, surface_normal)
        
        if penetration_depth > 0:
            in_contact = True
            
            spring_force = -self.contact_stiffness * penetration_depth * surface_normal
            
            damping_force = -self.contact_damping * np.dot(tool_velocity, surface_normal) * surface_normal
            
            contact_force = spring_force + damping_force
        else:
            in_contact = False
            contact_force = np.zeros(3)
        
        return contact_force, in_contact
    
    def plan_surface_following_trajectory(self, start_point: np.ndarray,
                                         end_point: np.ndarray,
                                         num_waypoints: int = 30,
                                         offset_distance: float = 0.002) -> List[Tuple[np.ndarray, np.ndarray]]:
        trajectory = []
        
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            point_3d = (1 - t) * start_point + t * end_point
            
            surface_point, surface_normal = self.tissue.get_surface_at_point(point_3d)
            
            tool_position = surface_point + offset_distance * surface_normal
            
            tool_orientation = -surface_normal
            
            trajectory.append((tool_position, tool_orientation))
        
        return trajectory
    
    def execute_incision(self, trajectory: List[Tuple[np.ndarray, np.ndarray]],
                        cutting_force: float = 1.0) -> List[bool]:
        cut_results = []
        
        for tool_position, tool_orientation in trajectory:
            self.tissue.apply_tool_contact(
                tool_position=tool_position,
                tool_orientation=tool_orientation,
                contact_force=cutting_force
            )
            
            cut_successful = self.tissue.apply_cutting(
                tool_position=tool_position,
                tool_orientation=tool_orientation,
                cutting_force=cutting_force
            )
            
            cut_results.append(cut_successful)
            
            self.tissue.step(use_implicit=False)
        
        return cut_results

def create_default_surgical_environment(position: np.ndarray = None) -> SurgicalTissueEnvironment:
    if position is None:
        position = np.array([0.3, 0.0, 0.25])
    
    tissue_env = SurgicalTissueEnvironment(
        tissue_type='multilayer',
        position=position
    )
    
    return tissue_env