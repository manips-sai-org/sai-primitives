import argparse
import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os 

def read_obj(file_path):
    """Reads an OBJ file and extracts unique surface vertices."""
    vertices = set()

    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if parts and parts[0] == 'v':  # Process only vertex lines
                vertex = tuple(map(float, parts[1:4]))
                vertices.add(vertex)

    return np.array(list(vertices))

def write_dat(vertices, output_file):
    """Writes vertices to a .dat file in the specified format."""
    with open(output_file, 'w') as f:
        f.write(f"{len(vertices)}\n")
        for v in vertices:
            f.write(f"{v[0]} {v[1]} {v[2]}\n")

def visualize_obj_and_vertices(obj_file, vertices):
    """Visualizes the OBJ mesh and extracted vertices with spheres."""
    # Load OBJ mesh
    mesh = pv.read(obj_file)

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

# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description="Extract unique surface vertices from an OBJ file and save them in a .dat file.")
#     parser.add_argument("obj_file", type=str, help="Path to the input OBJ file.")
#     parser.add_argument("dat_file", type=str, help="Path to the output .dat file.")

#     args = parser.parse_args()

#     # Process OBJ
#     vertices = read_obj(args.obj_file)
#     write_dat(vertices, args.dat_file)
#     print(f"Saved {len(vertices)} unique vertices to {args.dat_file}")

#     # Visualize results
#     visualize_obj_and_vertices(args.obj_file, vertices)
#     visualize_vertices_matplotlib(vertices)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process all OBJ files in a folder.")
    parser.add_argument("folder", type=str, help="Path to the folder containing OBJ files.")
    parser.add_argument("--visualize", action="store_true", help="Visualize the results using PyVista and Matplotlib.")

    args = parser.parse_args()

    obj_files = [f for f in os.listdir(args.folder) if f.lower().endswith('.obj')]
    obj_files.sort()  # Optional: process in alphabetical order

    if not obj_files:
        print("No OBJ files found in the specified folder.")
        exit(1)

    for obj_file in obj_files:
        obj_path = os.path.join(args.folder, obj_file)
        dat_path = os.path.splitext(obj_path)[0] + ".dat"

        print(f"Processing: {obj_file}")
        vertices = read_obj(obj_path)
        write_dat(vertices, dat_path)
        print(f"  → Saved {len(vertices)} unique vertices to {os.path.basename(dat_path)}")

        if args.visualize:
            visualize_obj_and_vertices(obj_path, vertices)
            visualize_vertices_matplotlib(vertices)