import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tissue_integration import create_default_surgical_environment, RobotTissueInteraction
from tissue_mesh import create_box_mesh
from tissue_materials import get_material_model
from tissue_fem import FEMTissueSimulator

def demo_simple_tissue_deformation():
    print("=" * 60)
    print("DEMO 1: Simple Tissue Deformation")
    print("=" * 60)
    
    mesh = create_box_mesh(
        dimensions=(0.03, 0.03, 0.01),
        resolution=(4, 4, 3),
        material_id=0
    )
    
    material = get_material_model('dermis', 'linear')
    
    simulator = FEMTissueSimulator(
        mesh=mesh,
        material_models=[material],
        dt=0.0001,
        gravity=np.array([0, 0, 0])
    )
    
    bottom_nodes = []
    z_min = np.min(mesh.nodes[:, 2])
    for i, node in enumerate(mesh.nodes):
        if abs(node[2] - z_min) < 0.0001:
            bottom_nodes.append(i)
    
    simulator.set_fixed_nodes(bottom_nodes)
    
    top_center_node = None
    z_max = np.max(mesh.nodes[:, 2])
    x_center = np.mean(mesh.nodes[:, 0])
    y_center = np.mean(mesh.nodes[:, 1])
    
    min_dist = float('inf')
    for i, node in enumerate(mesh.nodes):
        if abs(node[2] - z_max) < 0.0001:
            dist = np.sqrt((node[0] - x_center)**2 + (node[1] - y_center)**2)
            if dist < min_dist:
                min_dist = dist
                top_center_node = i
    
    print(f"Mesh: {mesh.n_nodes} nodes, {mesh.n_elements} elements")
    print(f"Fixed nodes: {len(bottom_nodes)}")
    print(f"Applying downward force to node {top_center_node}")
    print(f"Time step: {simulator.dt*1000:.3f} ms")
    
    fig = plt.figure(figsize=(12, 5))
    
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title('Initial Configuration')
    nodes, triangles = mesh.get_surface_mesh()
    for tri in triangles[:50]:
        verts = [nodes[tri[0]], nodes[tri[1]], nodes[tri[2]]]
        ax1.add_collection3d(Poly3DCollection([verts], alpha=0.3, facecolor='cyan', edgecolor='blue'))
    ax1.scatter(nodes[:, 0], nodes[:, 1], nodes[:, 2], c='blue', s=10)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_xlim([0, 0.03])
    ax1.set_ylim([0, 0.03])
    ax1.set_zlim([0, 0.012])
    
    force_magnitude = -0.2
    
    print("Simulating deformation...")
    for step in range(1000):
        simulator.reset_external_forces()
        simulator.apply_point_force(top_center_node, np.array([0, 0, force_magnitude]))
        simulator.step_explicit_euler()
        
        if step % 200 == 0:
            energy = simulator.get_strain_energy()
            ke = simulator.get_kinetic_energy()
            max_disp = np.max(np.abs(simulator.displacements))
            print(f"Step {step}: SE = {energy:.6f} J, KE = {ke:.6f} J, Max disp = {max_disp*1000:.3f} mm")
    
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set_title(f'Deformed (force = {abs(force_magnitude):.1f} N)')
    deformed_mesh = simulator.get_deformed_mesh()
    nodes, triangles = deformed_mesh.get_surface_mesh()
    for tri in triangles[:50]:
        verts = [nodes[tri[0]], nodes[tri[1]], nodes[tri[2]]]
        ax2.add_collection3d(Poly3DCollection([verts], alpha=0.3, facecolor='red', edgecolor='darkred'))
    ax2.scatter(nodes[:, 0], nodes[:, 1], nodes[:, 2], c='red', s=10)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.set_xlim([0, 0.03])
    ax2.set_ylim([0, 0.03])
    ax2.set_zlim([0, 0.012])
    
    plt.tight_layout()
    plt.savefig('./tissue_deformation_demo_fixed.png', dpi=150)
    print("\n✓ Saved visualization to tissue_deformation_demo_fixed.png\n")

def demo_multilayer_tissue():
    print("=" * 60)
    print("DEMO 2: Multi-Layer Tissue Simulation")
    print("=" * 60)
    
    from tissue_mesh import create_box_mesh
    from tissue_materials import get_material_model
    from tissue_fem import FEMTissueSimulator
    
    mesh = create_box_mesh(
        dimensions=(0.03, 0.03, 0.01),
        resolution=(4, 4, 3),
        material_id=0
    )
    
    material = get_material_model('dermis', 'linear')
    
    simulator = FEMTissueSimulator(
        mesh=mesh,
        material_models=[material],
        dt=0.0001,
        gravity=np.array([0, 0, 0])
    )
    
    bottom_nodes = []
    z_min = np.min(mesh.nodes[:, 2])
    for i, node in enumerate(mesh.nodes):
        if abs(node[2] - z_min) < 0.0001:
            bottom_nodes.append(i)
    
    simulator.set_fixed_nodes(bottom_nodes)
    
    print(f"Tissue mesh: {mesh.n_nodes} nodes, {mesh.n_elements} elements")
    print(f"Time step: {simulator.dt*1000:.3f} ms")
    
    center_point_idx = None
    z_max = np.max(mesh.nodes[:, 2])
    x_center = np.mean(mesh.nodes[:, 0])
    y_center = np.mean(mesh.nodes[:, 1])
    
    min_dist = float('inf')
    for i, node in enumerate(mesh.nodes):
        if abs(node[2] - z_max) < 0.0001:
            dist = np.sqrt((node[0] - x_center)**2 + (node[1] - y_center)**2)
            if dist < min_dist:
                min_dist = dist
                center_point_idx = i
    
    fig = plt.figure(figsize=(15, 5))
    
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.set_title('Initial State')
    nodes, triangles = mesh.get_surface_mesh()
    for tri in triangles[:50]:
        verts = [nodes[tri[0]], nodes[tri[1]], nodes[tri[2]]]
        ax1.add_collection3d(Poly3DCollection([verts], alpha=0.3, facecolor='tan', edgecolor='brown'))
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_xlim([0, 0.03])
    ax1.set_ylim([0, 0.03])
    ax1.set_zlim([0, 0.012])
    
    print("Simulating indentation...")
    for step in range(1000):
        simulator.reset_external_forces()
        simulator.apply_point_force(center_point_idx, np.array([0, 0, -0.15]))
        simulator.step_explicit_euler()
        
        if step % 200 == 0:
            max_disp = np.max(np.abs(simulator.displacements))
            energy = simulator.get_strain_energy()
            print(f"  Step {step}: Max displacement = {max_disp*1000:.3f} mm, Energy = {energy:.6f} J")
    
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.set_title('After Indentation')
    deformed_mesh = simulator.get_deformed_mesh()
    nodes, triangles = deformed_mesh.get_surface_mesh()
    for tri in triangles[:50]:
        verts = [nodes[tri[0]], nodes[tri[1]], nodes[tri[2]]]
        ax2.add_collection3d(Poly3DCollection([verts], alpha=0.3, facecolor='pink', edgecolor='red'))
    center_pos = deformed_mesh.nodes[center_point_idx]
    ax2.scatter([center_pos[0]], [center_pos[1]], [center_pos[2]], c='red', s=100, marker='*')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.set_xlim([0, 0.03])
    ax2.set_ylim([0, 0.03])
    ax2.set_zlim([0, 0.012])
    
    ax3 = fig.add_subplot(133)
    ax3.set_title('Displacement Magnitude')
    displacement = np.linalg.norm(simulator.displacements.reshape(-1, 3), axis=1)
    initial_nodes = mesh.nodes
    max_disp_viz = np.max(displacement) * 1.2
    sc = ax3.scatter(initial_nodes[:, 0]*1000, initial_nodes[:, 1]*1000, 
                    c=displacement*1000, cmap='hot', s=50, vmin=0, vmax=max_disp_viz*1000)
    plt.colorbar(sc, ax=ax3, label='Displacement (mm)')
    ax3.set_xlabel('X (mm)')
    ax3.set_ylabel('Y (mm)')
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('./multilayer_tissue_demo_fixed.png', dpi=150)
    energy = simulator.get_strain_energy()
    print(f"Final Strain Energy: {energy:.6f} J")
    print("\n✓ Saved visualization to multilayer_tissue_demo_fixed.png\n")

def demo_tissue_cutting():
    print("=" * 60)
    print("DEMO 3: Tissue Surface Trajectory Planning")
    print("=" * 60)
    
    from tissue_mesh import create_box_mesh
    from tissue_materials import get_material_model
    from tissue_fem import FEMTissueSimulator
    
    mesh = create_box_mesh(
        dimensions=(0.10, 0.10, 0.01),
        resolution=(8, 8, 3),
        material_id=0
    )
    
    material = get_material_model('dermis', 'linear')
    
    simulator = FEMTissueSimulator(
        mesh=mesh,
        material_models=[material],
        dt=0.0001,
        gravity=np.array([0, 0, 0])
    )
    
    bottom_nodes = []
    z_min = np.min(mesh.nodes[:, 2])
    for i, node in enumerate(mesh.nodes):
        if abs(node[2] - z_min) < 0.0001:
            bottom_nodes.append(i)
    
    simulator.set_fixed_nodes(bottom_nodes)
    
    start_point = np.array([0.02, 0.05, 0.012])
    end_point = np.array([0.08, 0.05, 0.012])
    
    num_waypoints = 20
    trajectory = []
    for i in range(num_waypoints):
        t = i / (num_waypoints - 1)
        point = (1 - t) * start_point + t * end_point
        trajectory.append(point)
    
    print(f"Planned trajectory: {len(trajectory)} waypoints")
    
    fig = plt.figure(figsize=(12, 6))
    
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title('Planned Incision Path')
    
    nodes, triangles = mesh.get_surface_mesh()
    for tri in triangles[:50]:
        verts = [nodes[tri[0]], nodes[tri[1]], nodes[tri[2]]]
        ax1.add_collection3d(Poly3DCollection([verts], alpha=0.2, facecolor='tan', edgecolor='brown'))
    
    path_points = np.array(trajectory)
    ax1.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], 
            'r-', linewidth=3, marker='o', label='Incision Path')
    ax1.legend()
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    
    z_max = np.max(mesh.nodes[:, 2])
    contact_nodes = []
    for i, node in enumerate(mesh.nodes):
        if abs(node[2] - z_max) < 0.0001:
            if 0.02 <= node[0] <= 0.08 and abs(node[1] - 0.05) < 0.02:
                contact_nodes.append(i)
    
    print(f"Simulating tool contact along path...")
    for step in range(100):
        simulator.reset_external_forces()
        for node_idx in contact_nodes:
            simulator.apply_point_force(node_idx, np.array([0, 0, -0.01]))
        simulator.step_explicit_euler()
    
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set_title('After Tool Contact')
    
    deformed_mesh = simulator.get_deformed_mesh()
    nodes, triangles = deformed_mesh.get_surface_mesh()
    for tri in triangles[:50]:
        verts = [nodes[tri[0]], nodes[tri[1]], nodes[tri[2]]]
        ax2.add_collection3d(Poly3DCollection([verts], alpha=0.2, facecolor='pink', edgecolor='red'))
    
    ax2.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], 
            'r--', linewidth=2, alpha=0.5, label='Original Path')
    ax2.legend()
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    
    plt.tight_layout()
    plt.savefig('./tissue_cutting_demo.png', dpi=150)
    print("\n✓ Saved visualization to tissue_cutting_demo.png\n")

def main():
    print("\n" + "=" * 60)
    print("FEM TISSUE SIMULATION DEMONSTRATIONS")
    print("=" * 60 + "\n")
    
    try:
        demo_simple_tissue_deformation()
    except Exception as e:
        print(f"Demo 1 failed: {e}")
    
    try:
        demo_multilayer_tissue()
    except Exception as e:
        print(f"Demo 2 failed: {e}")
    
    try:
        demo_tissue_cutting()
    except Exception as e:
        print(f"Demo 3 failed: {e}")
    
    print("=" * 60)
    print("All demonstrations complete!")
    print("=" * 60)

if __name__ == '__main__':
    main()