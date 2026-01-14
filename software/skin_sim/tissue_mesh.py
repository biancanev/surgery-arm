import numpy as np
from scipy.spatial import Delaunay
from dataclasses import dataclass
from typing import List, Set, Tuple, Optional

@dataclass
class TetrahedralElement:
    node_indices: np.ndarray
    material_id: int
    volume: float
    is_cut: bool = False
    virtual_nodes: Optional[List[int]] = None
    
    def __hash__(self):
        return hash(tuple(self.node_indices))

class TissueMesh:
    def __init__(self, nodes: np.ndarray, elements: List[TetrahedralElement]):
        self.nodes = nodes
        self.elements = elements
        self.n_nodes = len(nodes)
        self.n_elements = len(elements)
        
        self.surface_triangles = self._extract_surface()
        self.node_elements = self._build_node_element_map()
        self.element_neighbors = self._build_element_neighbors()
        
    def _extract_surface(self) -> List[Tuple[int, int, int]]:
        face_count = {}
        
        for elem in self.elements:
            faces = [
                tuple(sorted([elem.node_indices[0], elem.node_indices[1], elem.node_indices[2]])),
                tuple(sorted([elem.node_indices[0], elem.node_indices[1], elem.node_indices[3]])),
                tuple(sorted([elem.node_indices[0], elem.node_indices[2], elem.node_indices[3]])),
                tuple(sorted([elem.node_indices[1], elem.node_indices[2], elem.node_indices[3]])),
            ]
            for face in faces:
                face_count[face] = face_count.get(face, 0) + 1
        
        surface = [face for face, count in face_count.items() if count == 1]
        return surface
    
    def _build_node_element_map(self) -> List[Set[int]]:
        node_elements = [set() for _ in range(self.n_nodes)]
        for i, elem in enumerate(self.elements):
            for node_idx in elem.node_indices:
                node_elements[node_idx].add(i)
        return node_elements
    
    def _build_element_neighbors(self) -> List[Set[int]]:
        neighbors = [set() for _ in range(self.n_elements)]
        
        for i, elem_i in enumerate(self.elements):
            nodes_i = set(elem_i.node_indices)
            for j, elem_j in enumerate(self.elements):
                if i != j:
                    nodes_j = set(elem_j.node_indices)
                    if len(nodes_i & nodes_j) >= 3:
                        neighbors[i].add(j)
        
        return neighbors
    
    def get_element_nodes(self, elem_idx: int) -> np.ndarray:
        elem = self.elements[elem_idx]
        return self.nodes[elem.node_indices]
    
    def compute_element_volume(self, elem_idx: int) -> float:
        nodes = self.get_element_nodes(elem_idx)
        v1 = nodes[1] - nodes[0]
        v2 = nodes[2] - nodes[0]
        v3 = nodes[3] - nodes[0]
        return abs(np.dot(v1, np.cross(v2, v3))) / 6.0
    
    def get_surface_mesh(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self.surface_triangles:
            return np.array([]), np.array([])
        
        triangles = np.array(self.surface_triangles)
        return self.nodes, triangles

def create_box_mesh(dimensions: Tuple[float, float, float], 
                   resolution: Tuple[int, int, int],
                   material_id: int = 0) -> TissueMesh:
    lx, ly, lz = dimensions
    nx, ny, nz = resolution
    
    x = np.linspace(0, lx, nx)
    y = np.linspace(0, ly, ny)
    z = np.linspace(0, lz, nz)
    
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')
    nodes = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
    
    elements = []
    
    def get_node_index(i, j, k):
        return i * ny * nz + j * nz + k
    
    for i in range(nx - 1):
        for j in range(ny - 1):
            for k in range(nz - 1):
                n000 = get_node_index(i, j, k)
                n100 = get_node_index(i+1, j, k)
                n010 = get_node_index(i, j+1, k)
                n110 = get_node_index(i+1, j+1, k)
                n001 = get_node_index(i, j, k+1)
                n101 = get_node_index(i+1, j, k+1)
                n011 = get_node_index(i, j+1, k+1)
                n111 = get_node_index(i+1, j+1, k+1)
                
                tets = [
                    [n000, n100, n010, n001],
                    [n100, n110, n010, n111],
                    [n100, n001, n101, n111],
                    [n010, n001, n011, n111],
                    [n100, n010, n001, n111]
                ]
                
                for tet in tets:
                    elem = TetrahedralElement(
                        node_indices=np.array(tet, dtype=np.int32),
                        material_id=material_id,
                        volume=0.0
                    )
                    elements.append(elem)
    
    mesh = TissueMesh(nodes, elements)
    
    valid_elements = []
    for i, elem in enumerate(mesh.elements):
        elem.volume = mesh.compute_element_volume(i)
        if elem.volume > 1e-12:
            valid_elements.append(elem)
    
    mesh.elements = valid_elements
    mesh.n_elements = len(valid_elements)
    mesh.surface_triangles = mesh._extract_surface()
    
    return mesh

def create_layered_tissue_mesh(layer_specs: List[dict], 
                              xy_resolution: Tuple[int, int]) -> TissueMesh:
    nx, ny = xy_resolution
    
    all_nodes = []
    all_elements = []
    node_offset = 0
    
    z_current = 0.0
    
    for layer_id, spec in enumerate(layer_specs):
        thickness = spec['thickness']
        nz = spec.get('z_resolution', max(2, int(thickness * 10)))
        
        layer_mesh = create_box_mesh(
            dimensions=(spec['width'], spec['height'], thickness),
            resolution=(nx, ny, nz),
            material_id=layer_id
        )
        
        layer_mesh.nodes[:, 2] += z_current
        
        layer_mesh.nodes[:, 0] += spec.get('x_offset', 0)
        layer_mesh.nodes[:, 1] += spec.get('y_offset', 0)
        
        for elem in layer_mesh.elements:
            elem.node_indices += node_offset
        
        all_nodes.append(layer_mesh.nodes)
        all_elements.extend(layer_mesh.elements)
        
        node_offset += len(layer_mesh.nodes)
        z_current += thickness
    
    combined_nodes = np.vstack(all_nodes)
    
    return TissueMesh(combined_nodes, all_elements)

def compute_shape_functions(xi: float, eta: float, zeta: float) -> np.ndarray:
    N = np.array([
        1 - xi - eta - zeta,
        xi,
        eta,
        zeta
    ])
    return N

def compute_shape_function_derivatives() -> np.ndarray:
    dN_dxi = np.array([
        [-1, -1, -1],
        [ 1,  0,  0],
        [ 0,  1,  0],
        [ 0,  0,  1]
    ])
    return dN_dxi

def compute_jacobian(nodes: np.ndarray) -> np.ndarray:
    dN_dxi = compute_shape_function_derivatives()
    J = nodes.T @ dN_dxi
    return J

def compute_B_matrix(nodes: np.ndarray) -> np.ndarray:
    J = compute_jacobian(nodes)
    J_inv = np.linalg.inv(J)
    
    dN_dxi = compute_shape_function_derivatives()
    dN_dx = dN_dxi @ J_inv.T
    
    B = np.zeros((6, 12))
    
    for i in range(4):
        B[0, 3*i] = dN_dx[i, 0]
        B[1, 3*i+1] = dN_dx[i, 1]
        B[2, 3*i+2] = dN_dx[i, 2]
        
        B[3, 3*i] = dN_dx[i, 1]
        B[3, 3*i+1] = dN_dx[i, 0]
        
        B[4, 3*i+1] = dN_dx[i, 2]
        B[4, 3*i+2] = dN_dx[i, 1]
        
        B[5, 3*i] = dN_dx[i, 2]
        B[5, 3*i+2] = dN_dx[i, 0]
    
    return B