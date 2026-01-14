import numpy as np
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass
from tissue_mesh import TissueMesh
from tissue_fem import FEMTissueSimulator

@dataclass
class CutPlane:
    point: np.ndarray
    normal: np.ndarray
    
    def signed_distance(self, point: np.ndarray) -> float:
        return np.dot(point - self.point, self.normal)
    
    def intersects_segment(self, p0: np.ndarray, p1: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        d0 = self.signed_distance(p0)
        d1 = self.signed_distance(p1)
        
        if d0 * d1 < 0:
            t = d0 / (d0 - d1)
            intersection = p0 + t * (p1 - p0)
            return True, intersection
        
        return False, None

@dataclass
class CutEdge:
    node_a: int
    node_b: int
    intersection_point: np.ndarray
    
    def __hash__(self):
        return hash((min(self.node_a, self.node_b), max(self.node_a, self.node_b)))
    
    def __eq__(self, other):
        return (self.node_a, self.node_b) == (other.node_a, other.node_b) or \
               (self.node_b, self.node_a) == (other.node_a, other.node_b)

class TissueCuttingSimulator:
    def __init__(self, fem_simulator: FEMTissueSimulator):
        self.fem = fem_simulator
        self.mesh = fem_simulator.mesh
        
        self.cut_elements: Set[int] = set()
        self.virtual_nodes: List[np.ndarray] = []
        self.cut_surfaces: List[List[np.ndarray]] = []
        
        self.element_stress_cache = {}
    
    def apply_scalpel_cut(self, blade_position: np.ndarray, 
                         blade_direction: np.ndarray,
                         blade_length: float,
                         cutting_force: float) -> bool:
        blade_normal = blade_direction / np.linalg.norm(blade_direction)
        
        cut_plane = CutPlane(point=blade_position, normal=blade_normal)
        
        affected_elements = self._find_affected_elements(cut_plane, blade_length)
        
        cutting_successful = False
        
        for elem_idx in affected_elements:
            if self._can_cut_element(elem_idx, cutting_force):
                self._cut_element(elem_idx, cut_plane)
                cutting_successful = True
        
        return cutting_successful
    
    def _find_affected_elements(self, cut_plane: CutPlane, 
                                blade_length: float) -> List[int]:
        affected = []
        
        for elem_idx, elem in enumerate(self.mesh.elements):
            nodes = self.mesh.get_element_nodes(elem_idx)
            
            centroid = np.mean(nodes, axis=0)
            dist_to_plane = abs(cut_plane.signed_distance(centroid))
            
            if dist_to_plane < blade_length:
                distances = [cut_plane.signed_distance(node) for node in nodes]
                
                if min(distances) * max(distances) < 0:
                    affected.append(elem_idx)
        
        return affected
    
    def _can_cut_element(self, elem_idx: int, cutting_force: float) -> bool:
        elem = self.mesh.elements[elem_idx]
        material = self.fem.material_models[elem.material_id]
        
        if elem_idx not in self.element_stress_cache:
            stress = self._compute_element_stress(elem_idx)
            self.element_stress_cache[elem_idx] = stress
        else:
            stress = self.element_stress_cache[elem_idx]
        
        von_mises = self._compute_von_mises_stress(stress)
        
        stress_criterion = von_mises < material.properties.yield_stress
        
        force_criterion = cutting_force > material.properties.toughness * 0.001
        
        return stress_criterion and force_criterion
    
    def _compute_element_stress(self, elem_idx: int) -> np.ndarray:
        elem = self.mesh.elements[elem_idx]
        material = self.fem.material_models[elem.material_id]
        
        nodes = self.mesh.get_element_nodes(elem_idx)
        u_elem = np.zeros((4, 3))
        for i, node_idx in enumerate(elem.node_indices):
            u_elem[i] = self.fem.displacements[3*node_idx:3*node_idx+3]
        
        F = self.fem._compute_deformation_gradient(nodes, u_elem)
        P = material.compute_stress(F)
        
        return P
    
    def _compute_von_mises_stress(self, P: np.ndarray) -> float:
        sigma = P
        
        s11 = sigma[0, 0]
        s22 = sigma[1, 1]
        s33 = sigma[2, 2]
        s12 = sigma[0, 1]
        s23 = sigma[1, 2]
        s13 = sigma[0, 2]
        
        von_mises = np.sqrt(
            0.5 * ((s11 - s22)**2 + (s22 - s33)**2 + (s33 - s11)**2 +
                   6 * (s12**2 + s23**2 + s13**2))
        )
        
        return von_mises
    
    def _cut_element(self, elem_idx: int, cut_plane: CutPlane):
        elem = self.mesh.elements[elem_idx]
        nodes = self.mesh.get_element_nodes(elem_idx)
        
        cut_edges = []
        
        edges = [
            (0, 1), (0, 2), (0, 3),
            (1, 2), (1, 3), (2, 3)
        ]
        
        for i, j in edges:
            node_a = elem.node_indices[i]
            node_b = elem.node_indices[j]
            
            intersects, intersection = cut_plane.intersects_segment(nodes[i], nodes[j])
            
            if intersects:
                cut_edge = CutEdge(node_a, node_b, intersection)
                cut_edges.append(cut_edge)
        
        if len(cut_edges) >= 3:
            elem.is_cut = True
            self.cut_elements.add(elem_idx)
            
            cut_surface_points = [edge.intersection_point for edge in cut_edges]
            self.cut_surfaces.append(cut_surface_points)
            
            self._create_virtual_nodes(elem_idx, cut_plane)
    
    def _create_virtual_nodes(self, elem_idx: int, cut_plane: CutPlane):
        elem = self.mesh.elements[elem_idx]
        nodes = self.mesh.get_element_nodes(elem_idx)
        
        above_nodes = []
        below_nodes = []
        
        for i, node in enumerate(nodes):
            if cut_plane.signed_distance(node) > 0:
                above_nodes.append(elem.node_indices[i])
            else:
                below_nodes.append(elem.node_indices[i])
        
        virtual_above = []
        for node_idx in below_nodes:
            virtual_node_idx = len(self.mesh.nodes) + len(self.virtual_nodes)
            self.virtual_nodes.append(self.mesh.nodes[node_idx].copy())
            virtual_above.append(virtual_node_idx)
        
        virtual_below = []
        for node_idx in above_nodes:
            virtual_node_idx = len(self.mesh.nodes) + len(self.virtual_nodes)
            self.virtual_nodes.append(self.mesh.nodes[node_idx].copy())
            virtual_below.append(virtual_node_idx)
        
        elem.virtual_nodes = {
            'above': virtual_above,
            'below': virtual_below,
            'cut_plane': cut_plane
        }
    
    def update_cut_separation(self, separation_speed: float = 0.001):
        for elem_idx in self.cut_elements:
            elem = self.mesh.elements[elem_idx]
            
            if elem.virtual_nodes is None:
                continue
            
            cut_plane = elem.virtual_nodes['cut_plane']
            
            for vnode_idx in elem.virtual_nodes['above']:
                local_idx = vnode_idx - len(self.mesh.nodes)
                self.virtual_nodes[local_idx] += cut_plane.normal * separation_speed * self.fem.dt
            
            for vnode_idx in elem.virtual_nodes['below']:
                local_idx = vnode_idx - len(self.mesh.nodes)
                self.virtual_nodes[local_idx] -= cut_plane.normal * separation_speed * self.fem.dt
    
    def get_cut_surface_geometry(self) -> List[np.ndarray]:
        all_surfaces = []
        
        for surface_points in self.cut_surfaces:
            if len(surface_points) >= 3:
                all_surfaces.append(np.array(surface_points))
        
        return all_surfaces
    
    def compute_cut_depth(self, reference_point: np.ndarray) -> float:
        if not self.cut_surfaces:
            return 0.0
        
        max_depth = 0.0
        
        for surface_points in self.cut_surfaces:
            for point in surface_points:
                depth = np.linalg.norm(point - reference_point)
                max_depth = max(max_depth, depth)
        
        return max_depth

class ProgressiveCuttingSimulator:
    def __init__(self, fem_simulator: FEMTissueSimulator):
        self.fem = fem_simulator
        self.mesh = fem_simulator.mesh
        
        self.crack_tips: List[Tuple[np.ndarray, np.ndarray]] = []
        self.crack_paths: List[List[np.ndarray]] = []
    
    def initiate_cut(self, blade_position: np.ndarray, 
                    blade_direction: np.ndarray,
                    cutting_force: float):
        blade_normal = blade_direction / np.linalg.norm(blade_direction)
        
        nearest_surface = self._find_nearest_surface_point(blade_position)
        
        self.crack_tips.append((nearest_surface, blade_normal))
        self.crack_paths.append([nearest_surface])
    
    def propagate_cuts(self, dt: float):
        for i, (tip_position, direction) in enumerate(self.crack_tips):
            K_I = self._compute_stress_intensity_factor(tip_position, direction)
            
            material = self.fem.material_models[0]
            K_Ic = np.sqrt(material.properties.toughness * material.properties.youngs_modulus)
            
            if K_I > K_Ic:
                crack_speed = 0.001
                propagation = direction * crack_speed * dt
                
                new_tip = tip_position + propagation
                
                self.crack_tips[i] = (new_tip, direction)
                self.crack_paths[i].append(new_tip)
    
    def _compute_stress_intensity_factor(self, crack_tip: np.ndarray, 
                                        direction: np.ndarray) -> float:
        nearest_elem = self._find_nearest_element(crack_tip)
        
        if nearest_elem is None:
            return 0.0
        
        elem = self.mesh.elements[nearest_elem]
        material = self.fem.material_models[elem.material_id]
        
        nodes = self.mesh.get_element_nodes(nearest_elem)
        u_elem = np.zeros((4, 3))
        for i, node_idx in enumerate(elem.node_indices):
            u_elem[i] = self.fem.displacements[3*node_idx:3*node_idx+3]
        
        F = self.fem._compute_deformation_gradient(nodes, u_elem)
        P = material.compute_stress(F)
        
        sigma = np.linalg.norm(P)
        
        a = 0.001
        K_I = sigma * np.sqrt(np.pi * a)
        
        return K_I
    
    def _find_nearest_surface_point(self, point: np.ndarray) -> np.ndarray:
        min_dist = float('inf')
        nearest = point
        
        for triangle in self.mesh.surface_triangles:
            n0, n1, n2 = triangle
            centroid = (self.mesh.nodes[n0] + self.mesh.nodes[n1] + self.mesh.nodes[n2]) / 3
            
            dist = np.linalg.norm(point - centroid)
            if dist < min_dist:
                min_dist = dist
                nearest = centroid
        
        return nearest
    
    def _find_nearest_element(self, point: np.ndarray) -> Optional[int]:
        min_dist = float('inf')
        nearest_elem = None
        
        for i, elem in enumerate(self.mesh.elements):
            nodes = self.mesh.get_element_nodes(i)
            centroid = np.mean(nodes, axis=0)
            
            dist = np.linalg.norm(point - centroid)
            if dist < min_dist:
                min_dist = dist
                nearest_elem = i
        
        return nearest_elem