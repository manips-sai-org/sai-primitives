import argparse
import numpy as np
import struct
import pyvista as pv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_stl(file_path):
    """Reads an STL file and extracts unique surface vertices."""
    vertices = set()
    
    with open(file_path, 'rb') as f:
        f.read(80)  # Skip header
        num_triangles = struct.unpack('<I', f.read(4))[0]

        for _ in range(num_triangles):
            f.read(12)  # Skip normal vector
            for _ in range(3):  # Read three vertices
                vertex = struct.unpack('<3f', f.read(12))
                vertices.add(vertex)
            f.read(2)  # Skip attribute byte count

    return np.array(list(vertices))

def write_dat(vertices, output_file):
    """Writes vertices to a .dat file in the specified format."""
    with open(output_file, 'w') as f:
        f.write(f"{len(vertices)}\n")
        for v in vertices:
            f.write(f"{v[0]} {v[1]} {v[2]}\n")

def visualize_stl_and_vertices(stl_file, vertices):
    """Visualizes the STL mesh and extracted vertices with spheres."""
    # Load STL mesh
    mesh = pv.read(stl_file)

    # Create a PyVista plotter
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color="lightblue", opacity=0.5, show_edges=True)

    # Add extracted vertices as small spheres
    for v in vertices:
        sphere = pv.Sphere(radius=0.005, center=v)
        plotter.add_mesh(sphere, color="red")

    plotter.show()

def visualize_vertices_matplotlib(vertices):
    """Visualizes extracted vertices using Matplotlib."""
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], color='red', s=5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Extracted Surface Vertices')
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract unique surface vertices from an STL file and save them in a .dat file.")
    parser.add_argument("stl_file", type=str, help="Path to the input STL file.")
    parser.add_argument("dat_file", type=str, help="Path to the output .dat file.")

    args = parser.parse_args()

    # Process STL
    vertices = read_stl(args.stl_file)
    write_dat(vertices, args.dat_file)
    print(f"Saved {len(vertices)} unique vertices to {args.dat_file}")

    # Visualize results
    visualize_stl_and_vertices(args.stl_file, vertices)
    visualize_vertices_matplotlib(vertices)
