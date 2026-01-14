import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass

@dataclass
class MaterialProperties:
    name: str
    density: float
    youngs_modulus: float
    poisson_ratio: float
    yield_stress: float
    toughness: float
    damping: float = 0.1
    
    def lame_parameters(self) -> tuple:
        E = self.youngs_modulus
        nu = self.poisson_ratio
        lambda_lame = (E * nu) / ((1 + nu) * (1 - 2*nu))
        mu = E / (2 * (1 + nu))
        return lambda_lame, mu

class ConstitutiveModel(ABC):
    def __init__(self, properties: MaterialProperties):
        self.properties = properties
    
    @abstractmethod
    def compute_stress(self, F: np.ndarray) -> np.ndarray:
        pass
    
    @abstractmethod
    def compute_tangent_modulus(self, F: np.ndarray) -> np.ndarray:
        pass

class LinearElasticModel(ConstitutiveModel):
    def __init__(self, properties: MaterialProperties):
        super().__init__(properties)
        self.lambda_lame, self.mu = properties.lame_parameters()
    
    def compute_constitutive_matrix(self) -> np.ndarray:
        lambda_lame = self.lambda_lame
        mu = self.mu
        
        D = np.zeros((6, 6))
        
        D[0, 0] = D[1, 1] = D[2, 2] = lambda_lame + 2*mu
        D[0, 1] = D[0, 2] = D[1, 0] = D[1, 2] = D[2, 0] = D[2, 1] = lambda_lame
        D[3, 3] = D[4, 4] = D[5, 5] = mu
        
        return D
    
    def compute_stress(self, strain: np.ndarray) -> np.ndarray:
        D = self.compute_constitutive_matrix()
        stress = D @ strain
        return stress
    
    def compute_tangent_modulus(self, strain: np.ndarray) -> np.ndarray:
        return self.compute_constitutive_matrix()

class NeoHookeanModel(ConstitutiveModel):
    def __init__(self, properties: MaterialProperties):
        super().__init__(properties)
        self.lambda_lame, self.mu = properties.lame_parameters()
    
    def compute_stress(self, F: np.ndarray) -> np.ndarray:
        J = np.linalg.det(F)
        
        if J <= 0:
            J = 1e-10
        
        C = F.T @ F
        C_inv = np.linalg.inv(C)
        
        S = self.mu * (np.eye(3) - C_inv) + self.lambda_lame * np.log(J) * C_inv
        
        P = F @ S
        
        return P
    
    def compute_tangent_modulus(self, F: np.ndarray) -> np.ndarray:
        J = np.linalg.det(F)
        if J <= 0:
            J = 1e-10
        
        C = F.T @ F
        C_inv = np.linalg.inv(C)
        
        D_mat = np.zeros((6, 6))
        
        lambda_lame = self.lambda_lame
        mu = self.mu
        
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    for l in range(3):
                        idx_ij = self._voigt_index(i, j)
                        idx_kl = self._voigt_index(k, l)
                        
                        D_mat[idx_ij, idx_kl] = (
                            lambda_lame * C_inv[i,j] * C_inv[k,l] +
                            (mu - lambda_lame * np.log(J)) * (
                                C_inv[i,k] * C_inv[j,l] + C_inv[i,l] * C_inv[j,k]
                            )
                        )
        
        return D_mat
    
    def _voigt_index(self, i: int, j: int) -> int:
        if i == j:
            return i
        else:
            return 6 - i - j

class MooneyRivlinModel(ConstitutiveModel):
    def __init__(self, properties: MaterialProperties, C10: float = None, C01: float = None):
        super().__init__(properties)
        
        if C10 is None:
            mu = properties.youngs_modulus / (2 * (1 + properties.poisson_ratio))
            self.C10 = 0.6 * mu
            self.C01 = 0.4 * mu
        else:
            self.C10 = C10
            self.C01 = C01
        
        self.lambda_lame, _ = properties.lame_parameters()
    
    def compute_stress(self, F: np.ndarray) -> np.ndarray:
        J = np.linalg.det(F)
        if J <= 0:
            J = 1e-10
        
        C = F.T @ F
        C_inv = np.linalg.inv(C)
        
        I1 = np.trace(C)
        I2 = 0.5 * (I1**2 - np.trace(C @ C))
        
        C_bar = J**(-2/3) * C
        I1_bar = np.trace(C_bar)
        I2_bar = 0.5 * (I1_bar**2 - np.trace(C_bar @ C_bar))
        
        dW_dI1 = self.C10
        dW_dI2 = self.C01
        
        S = 2 * J**(-2/3) * (
            dW_dI1 * (np.eye(3) - I1_bar/3 * C_inv) +
            dW_dI2 * (I1_bar * np.eye(3) - C - 2*I2_bar/3 * C_inv)
        ) + self.lambda_lame * (J - 1) * J * C_inv
        
        P = F @ S
        
        return P
    
    def compute_tangent_modulus(self, F: np.ndarray) -> np.ndarray:
        return self.compute_numerical_tangent(F)
    
    def compute_numerical_tangent(self, F: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        D = np.zeros((6, 6))
        
        for i in range(6):
            F_plus = F.copy()
            F_minus = F.copy()
            
            if i < 3:
                F_plus[i, i] += eps
                F_minus[i, i] -= eps
            else:
                idx_map = {3: (0,1), 4: (1,2), 5: (0,2)}
                ii, jj = idx_map[i]
                F_plus[ii, jj] += eps
                F_minus[ii, jj] -= eps
            
            P_plus = self.compute_stress(F_plus)
            P_minus = self.compute_stress(F_minus)
            
            dP = (P_plus - P_minus) / (2 * eps)
            
            D[:, i] = self._tensor_to_voigt(dP)
        
        return D
    
    def _tensor_to_voigt(self, tensor: np.ndarray) -> np.ndarray:
        return np.array([
            tensor[0, 0],
            tensor[1, 1],
            tensor[2, 2],
            tensor[0, 1],
            tensor[1, 2],
            tensor[0, 2]
        ])

TISSUE_LIBRARY = {
    'epidermis': MaterialProperties(
        name='epidermis',
        density=1200.0,
        youngs_modulus=1.0e6,
        poisson_ratio=0.48,
        yield_stress=5.0e5,
        toughness=1000.0,
        damping=0.15
    ),
    'dermis': MaterialProperties(
        name='dermis',
        density=1100.0,
        youngs_modulus=5.0e5,
        poisson_ratio=0.48,
        yield_stress=2.0e5,
        toughness=800.0,
        damping=0.12
    ),
    'subcutaneous_fat': MaterialProperties(
        name='subcutaneous_fat',
        density=900.0,
        youngs_modulus=2.0e4,
        poisson_ratio=0.49,
        yield_stress=1.0e4,
        toughness=300.0,
        damping=0.20
    ),
    'muscle': MaterialProperties(
        name='muscle',
        density=1060.0,
        youngs_modulus=1.0e5,
        poisson_ratio=0.47,
        yield_stress=3.0e5,
        toughness=600.0,
        damping=0.10
    )
}

def get_material_model(material_name: str, model_type: str = 'neo_hookean') -> ConstitutiveModel:
    if material_name not in TISSUE_LIBRARY:
        raise ValueError(f"Unknown material: {material_name}")
    
    properties = TISSUE_LIBRARY[material_name]
    
    if model_type == 'linear':
        return LinearElasticModel(properties)
    elif model_type == 'neo_hookean':
        return NeoHookeanModel(properties)
    elif model_type == 'mooney_rivlin':
        return MooneyRivlinModel(properties)
    else:
        raise ValueError(f"Unknown model type: {model_type}")