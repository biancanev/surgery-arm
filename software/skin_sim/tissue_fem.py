import numpy as np
from scipy.sparse import lil_matrix, csr_matrix
from scipy.sparse.linalg import spsolve
from typing import List, Tuple, Optional
from tissue_mesh import TissueMesh, compute_B_matrix, compute_jacobian
from tissue_materials import ConstitutiveModel, get_material_model, LinearElasticModel

class FEMTissueSimulator:
    def __init__(self, mesh: TissueMesh, material_models: List[ConstitutiveModel], 
                 dt: float = 0.001, gravity: np.ndarray = np.array([0, 0, -9.81])):
        self.mesh = mesh
        self.material_models = material_models
        self.dt = dt
        self.gravity = gravity
        
        self.displacements = np.zeros(3 * mesh.n_nodes)
        self.velocities = np.zeros(3 * mesh.n_nodes)
        self.accelerations = np.zeros(3 * mesh.n_nodes)
        
        self.external_forces = np.zeros(3 * mesh.n_nodes)
        
        self.fixed_nodes = set()
        
        self.time = 0.0
        self.step = 0
        
        self.mass_matrix = self._assemble_mass_matrix()
        
        self._cache_element_data()
    
    def _cache_element_data(self):
        self.element_B_matrices = []
        self.element_volumes = []
        
        for i, elem in enumerate(self.mesh.elements):
            nodes = self.mesh.get_element_nodes(i)
            B = compute_B_matrix(nodes)
            self.element_B_matrices.append(B)
            self.element_volumes.append(elem.volume)
    
    def _assemble_mass_matrix(self) -> csr_matrix:
        n_dof = 3 * self.mesh.n_nodes
        M = lil_matrix((n_dof, n_dof))
        
        for elem_idx, elem in enumerate(self.mesh.elements):
            material = self.material_models[elem.material_id]
            rho = material.properties.density
            
            volume = elem.volume
            lumped_mass = rho * volume / 4.0
            
            for node in elem.node_indices:
                for i in range(3):
                    dof = 3 * node + i
                    M[dof, dof] += lumped_mass
        
        return M.tocsr()
    
    def _assemble_stiffness_matrix(self) -> csr_matrix:
        n_dof = 3 * self.mesh.n_nodes
        K = lil_matrix((n_dof, n_dof))
        
        for elem_idx, elem in enumerate(self.mesh.elements):
            K_elem = self._compute_element_stiffness(elem_idx)
            
            dofs = []
            for node in elem.node_indices:
                dofs.extend([3*node, 3*node+1, 3*node+2])
            
            for i, dof_i in enumerate(dofs):
                for j, dof_j in enumerate(dofs):
                    K[dof_i, dof_j] += K_elem[i, j]
        
        return K.tocsr()
    
    def _compute_element_stiffness(self, elem_idx: int) -> np.ndarray:
        elem = self.mesh.elements[elem_idx]
        material = self.material_models[elem.material_id]
        
        B = self.element_B_matrices[elem_idx]
        volume = self.element_volumes[elem_idx]
        
        nodes = self.mesh.get_element_nodes(elem_idx)
        u_elem = np.zeros((4, 3))
        for i, node_idx in enumerate(elem.node_indices):
            u_elem[i] = self.displacements[3*node_idx:3*node_idx+3]
        
        F = self._compute_deformation_gradient(nodes, u_elem)
        
        D = material.compute_tangent_modulus(F)
        
        K_elem = B.T @ D @ B * volume
        
        return K_elem
    
    def _compute_deformation_gradient(self, nodes: np.ndarray, 
                                     displacements: np.ndarray) -> np.ndarray:
        J = compute_jacobian(nodes)
        J_inv = np.linalg.inv(J)
        
        dN_dxi = np.array([
            [-1, -1, -1],
            [ 1,  0,  0],
            [ 0,  1,  0],
            [ 0,  0,  1]
        ])
        
        dN_dx = dN_dxi @ J_inv.T
        
        grad_u = displacements.T @ dN_dx
        
        F = np.eye(3) + grad_u
        
        return F
    
    def _compute_internal_forces(self) -> np.ndarray:
        f_int = np.zeros(3 * self.mesh.n_nodes)
        
        for elem_idx, elem in enumerate(self.mesh.elements):
            material = self.material_models[elem.material_id]
            
            nodes = self.mesh.get_element_nodes(elem_idx)
            u_elem = np.zeros(12)
            for i, node_idx in enumerate(elem.node_indices):
                u_elem[3*i:3*i+3] = self.displacements[3*node_idx:3*node_idx+3]
            
            B = self.element_B_matrices[elem_idx]
            volume = self.element_volumes[elem_idx]
            
            if isinstance(material, LinearElasticModel):
                strain_voigt = B @ u_elem
                
                stress_voigt = material.compute_stress(strain_voigt)
            else:
                F = self._compute_deformation_gradient(nodes, u_elem.reshape(4, 3))
                P = material.compute_stress(F)
                stress_voigt = self._pk1_to_voigt(P)
            
            f_elem = B.T @ stress_voigt * volume
            
            for i, node_idx in enumerate(elem.node_indices):
                for j in range(3):
                    dof = 3 * node_idx + j
                    f_int[dof] += f_elem[3*i + j]
        
        return f_int
    
    def _pk1_to_voigt(self, P: np.ndarray) -> np.ndarray:
        return np.array([
            P[0, 0],
            P[1, 1],
            P[2, 2],
            P[0, 1],
            P[1, 2],
            P[0, 2]
        ])
    
    def _compute_gravity_forces(self) -> np.ndarray:
        f_gravity = np.zeros(3 * self.mesh.n_nodes)
        
        for elem_idx, elem in enumerate(self.mesh.elements):
            material = self.material_models[elem.material_id]
            rho = material.properties.density
            volume = elem.volume
            
            force_per_node = rho * volume * self.gravity / 4.0
            
            for node_idx in elem.node_indices:
                for i in range(3):
                    dof = 3 * node_idx + i
                    f_gravity[dof] += force_per_node[i]
        
        return f_gravity
    
    def set_fixed_nodes(self, node_indices: List[int]):
        self.fixed_nodes = set(node_indices)
    
    def apply_boundary_conditions(self, matrix: csr_matrix, 
                                  vector: np.ndarray) -> Tuple[csr_matrix, np.ndarray]:
        matrix = matrix.tolil()
        
        for node in self.fixed_nodes:
            for i in range(3):
                dof = 3 * node + i
                matrix[dof, :] = 0
                matrix[:, dof] = 0
                matrix[dof, dof] = 1.0
                vector[dof] = 0.0
        
        return matrix.tocsr(), vector
    
    def step_implicit_euler(self):
        f_int = self._compute_internal_forces()
        f_gravity = self._compute_gravity_forces()
        f_ext = self.external_forces
        
        f_total = -f_int + f_gravity + f_ext
        
        K = self._assemble_stiffness_matrix()
        
        damping_factor = 0.01
        C = damping_factor * K
        
        effective_K = self.mass_matrix + self.dt * C + self.dt**2 * K
        effective_f = self.mass_matrix @ self.velocities + self.dt * f_total
        
        effective_K, effective_f = self.apply_boundary_conditions(effective_K, effective_f)
        
        try:
            new_velocities = spsolve(effective_K, effective_f)
        except:
            print("Warning: Solver failed, using previous velocities")
            new_velocities = self.velocities
        
        self.accelerations = (new_velocities - self.velocities) / self.dt
        self.velocities = new_velocities
        self.displacements += self.dt * self.velocities
        
        self.time += self.dt
        self.step += 1
    
    def step_explicit_euler(self):
        f_int = self._compute_internal_forces()
        f_gravity = self._compute_gravity_forces()
        f_ext = self.external_forces
        
        f_total = -f_int + f_gravity + f_ext
        
        M_diag = self.mass_matrix.diagonal()
        
        for node in self.fixed_nodes:
            for i in range(3):
                dof = 3 * node + i
                f_total[dof] = 0
                self.velocities[dof] = 0
        
        self.accelerations = f_total / np.maximum(M_diag, 1e-10)
        
        max_accel = np.max(np.abs(self.accelerations))
        if max_accel > 1e6:
            print(f"Warning: Large acceleration {max_accel:.2e}, clamping")
            self.accelerations = np.clip(self.accelerations, -1e6, 1e6)
        
        damping = 0.99
        self.velocities += self.dt * self.accelerations
        self.velocities *= damping
        
        max_vel = np.max(np.abs(self.velocities))
        if max_vel > 10.0:
            print(f"Warning: Large velocity {max_vel:.2e}, clamping")
            self.velocities = np.clip(self.velocities, -10.0, 10.0)
        
        self.displacements += self.dt * self.velocities
        
        max_disp = np.max(np.abs(self.displacements))
        if max_disp > 0.1:
            print(f"Warning: Large displacement {max_disp:.2e}, clamping")
            self.displacements = np.clip(self.displacements, -0.1, 0.1)
        
        self.time += self.dt
        self.step += 1
    
    def get_deformed_mesh(self) -> TissueMesh:
        deformed_nodes = self.mesh.nodes.copy()
        for i in range(self.mesh.n_nodes):
            deformed_nodes[i] += self.displacements[3*i:3*i+3]
        
        from copy import deepcopy
        deformed_mesh = deepcopy(self.mesh)
        deformed_mesh.nodes = deformed_nodes
        
        return deformed_mesh
    
    def apply_point_force(self, node_idx: int, force: np.ndarray):
        self.external_forces[3*node_idx:3*node_idx+3] = force
    
    def apply_surface_pressure(self, pressure: float):
        for triangle in self.mesh.surface_triangles:
            n0, n1, n2 = triangle
            
            p0 = self.mesh.nodes[n0]
            p1 = self.mesh.nodes[n1]
            p2 = self.mesh.nodes[n2]
            
            v1 = p1 - p0
            v2 = p2 - p0
            normal = np.cross(v1, v2)
            area = np.linalg.norm(normal) / 2.0
            normal = normal / (2.0 * area)
            
            force_per_node = pressure * area * normal / 3.0
            
            for node in triangle:
                self.external_forces[3*node:3*node+3] += force_per_node
    
    def reset_external_forces(self):
        self.external_forces = np.zeros(3 * self.mesh.n_nodes)
    
    def get_strain_energy(self) -> float:
        energy = 0.0
        
        for elem_idx, elem in enumerate(self.mesh.elements):
            material = self.material_models[elem.material_id]
            
            nodes = self.mesh.get_element_nodes(elem_idx)
            u_elem = np.zeros(12)
            for i, node_idx in enumerate(elem.node_indices):
                u_elem[3*i:3*i+3] = self.displacements[3*node_idx:3*node_idx+3]
            
            if isinstance(material, LinearElasticModel):
                B = self.element_B_matrices[elem_idx]
                strain_voigt = B @ u_elem
                stress_voigt = material.compute_stress(strain_voigt)
                
                W = 0.5 * np.dot(strain_voigt, stress_voigt)
            else:
                F = self._compute_deformation_gradient(nodes, u_elem.reshape(4, 3))
                
                C = F.T @ F
                I1 = np.trace(C)
                J = np.linalg.det(F)
                
                if J <= 0:
                    J = 1e-10
                
                lambda_lame, mu = material.properties.lame_parameters()
                W = (mu/2) * (I1 - 3) - mu * np.log(J) + (lambda_lame/2) * (np.log(J))**2
            
            energy += W * elem.volume
        
        return energy
    
    def get_kinetic_energy(self) -> float:
        v = self.velocities.reshape(-1, 3)
        masses = self.mass_matrix.diagonal().reshape(-1, 3)[:, 0]
        
        ke = 0.5 * np.sum(masses[:, np.newaxis] * v**2)
        return ke