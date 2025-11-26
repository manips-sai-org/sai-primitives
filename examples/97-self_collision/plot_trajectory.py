import pandas as pd
import matplotlib.pyplot as plt
import numpy as np 
import matplotlib 
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.collections as mcollections
import pywavefront
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # Correct import
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.collections import LineCollection
matplotlib.use('TkAgg')  # Alternative: 'Qt5Agg'

# Joint limits 
q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
offset = 0.5 

def set_axes_equal(ax):
    """Set equal scaling on all 3 axes of a 3D plot."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    max_range = max(x_range, y_range, z_range)

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    ax.set_xlim3d([x_middle - max_range/2, x_middle + max_range/2])
    ax.set_ylim3d([y_middle - max_range/2, y_middle + max_range/2])
    ax.set_zlim3d([z_middle - max_range/2, z_middle + max_range/2])

def equal_ratio(ax):
    """Set equal aspect ratio for 3D axes in matplotlib."""
    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()

    x_middle = np.mean(xlim)
    y_middle = np.mean(ylim)
    z_middle = np.mean(zlim)

    radius = 0.5 * max(xlim[1] - xlim[0],
                       ylim[1] - ylim[0],
                       zlim[1] - zlim[0])

    ax.set_xlim3d([x_middle - radius, x_middle + radius])
    ax.set_ylim3d([y_middle - radius, y_middle + radius])
    ax.set_zlim3d([z_middle - radius, z_middle + radius])

def load_obj(filepath):
    vertices = []
    faces = []
    with open(filepath, 'r') as file:
        for line in file:
            if line.startswith('v '):
                vertex = list(map(float, line.strip().split()[1:]))
                vertices.append(vertex)
            elif line.startswith('f '):
                face = [int(item.split('/')[0]) for item in line.strip().split()[1:]]
                faces.append(face)
    return np.array(vertices), np.array(faces) - 1 # OBJ indices start from 1

# Load CSV file
def plot_joint(csv_file):
    df = pd.read_csv(csv_file)
    
    # Drop 'timestamp' column if it exists
    if 'timestamp' in df.columns:
        df = df.drop(columns=['timestamp'])
    
    # Convert 'time' column to float
    df['time'] = df['time'].astype(float)
    
    # Extract joint columns
    ee_columns = [col for col in df.columns if ' ee_pos__' in col]
    ee_vel_columns = [col for col in df.columns if ' ee_vel__' in col]
    
    constraint_flag = df[' constraint_flag']
    
    # # Ensure joint_index is within range
    # if joint_index < 0 or joint_index >= len(joint_columns):
    #     raise ValueError("Invalid joint index")
    
    # joint = joint_columns[joint_index]
    
    # Plot the selected joint
    # plt.figure(figsize=(10, 6))
    # plt.plot(df['time'], df[ee_columns], label=joint)
    
    start_ind = 500  # method
    # start_ind = 500  # baseline
    end_ind = 1100 - 200
    method_x_offset = (0.399114 - 0.378128)
    method_z_offset = 0.207 * 1

    df[ee_columns[0]][:] += method_x_offset
    df[ee_columns[2]][:] += method_z_offset

    # starting position
    print("Starting position: ", df[ee_columns[0]][start_ind], df[ee_columns[1]][start_ind], df[ee_columns[2]][start_ind])

    # 0.378128 0.00421217 0.173724
    # 0.399114 0.000233295 0.38051
    
    # Create figure and subplots
    # fig, axes = plt.subplots(3, 1, figsize=(6, 10))
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))

    # First subplot
    axes[0].plot(df['time'][start_ind:end_ind], df[ee_columns[0]][start_ind:end_ind], label="X", color="b")
    axes[0].set_title("End-Effector X Position")
    axes[0].legend()
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('X (m)')
    axes[0].grid()
    # Plot rectangles where binary_vector == 1
    for i in range(len(constraint_flag)):
        if constraint_flag[i] == 1 and i > start_ind:
            # axes[0].add_patch(plt.Rectangle((df['time'][i], min(df[ee_columns[0]])), 0.5, max(df[ee_columns[0]]) - min(df[ee_columns[0]]), color='yellow', alpha=0.5))  # (x, y), width, height
            axes[0].axvspan(df['time'][i], df['time'][i+1], color='yellow', alpha=0.02)

    # Second subplot
    axes[1].plot(df['time'][start_ind:end_ind], df[ee_columns[1]][start_ind:end_ind], label="Y", color="r")
    axes[1].set_title("End-Effector Y Position")
    axes[1].legend()
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Y (m)')
    axes[1].grid()
    # Plot rectangles where binary_vector == 1
    for i in range(len(constraint_flag)):
        if constraint_flag[i] == 1 and i > start_ind:
            # axes[1].add_patch(plt.Rectangle((df['time'][i], min(df[ee_columns[1]])), 0.5, max(df[ee_columns[1]]) - min(df[ee_columns[1]]), color='yellow', alpha=0.5))  # (x, y), width, height
            axes[1].axvspan(df['time'][i], df['time'][i+1], color='yellow', alpha=0.02)

    # Third subplot
    axes[2].plot(df['time'][start_ind:end_ind], df[ee_columns[2]][start_ind:end_ind], label="Z", color="g")  # Clipping to avoid large values
    axes[2].set_title("End-Effector Z Position")
    axes[2].legend()
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Z (m)')
    axes[2].grid()
    # Plot rectangles where binary_vector == 1
    for i in range(len(constraint_flag)):
        if constraint_flag[i] == 1 and i > start_ind:
            # axes[2].add_patch(plt.Rectangle((df['time'][i], min(df[ee_columns[2]])), 0.5, max(df[ee_columns[2]]) - min(df[ee_columns[2]]), color='yellow', alpha=0.5))  # (x, y), width, height
            axes[2].axvspan(df['time'][i], df['time'][i+1], color='yellow', alpha=0.02)

    # Adjust layout and show plot
    # plt.show()
    
    # # Draw horizontal dashed lines
    # plt.axhline(y=q_min[joint_index] + offset, color='g', linestyle='--')
    # plt.axhline(y=q_max[joint_index] - offset, color='g', linestyle='--')
    # plt.axhline(y=q_min[joint_index], color='r', linestyle='--')
    # plt.axhline(y=q_max[joint_index], color='r', linestyle='--')
    
    # plt.xlabel('Time (s)')
    # plt.ylabel('EE Pos (m)')
    fig.tight_layout()
    # fig.suptitle("End-Effector Position During Self-Collision Handling")
    # plt.legend()
    # plt.grid()

    """ 
        EE spatial plot (x vs y), (y vs. z), (x vs. z)
    """
    # Example: assume df, ee_columns, and start_ind:end_ind are defined
    x = df[ee_columns[1]][start_ind:end_ind].values
    y = df[ee_columns[2]][start_ind:end_ind].values

    # Create segments for the line
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Use time or index as the color parameter
    t = np.linspace(0, 1, len(x) - 2)  # Normalized time

    # Create the LineCollection
    lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(0, 1))
    lc.set_array(t)
    lc.set_linewidth(2)

    # Normalize time for color mapping
    t = np.linspace(0, 1, len(x))  # or use timestamps normalized to [0, 1]

    # Plot ee position 
    # fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    # fig, axes = plt.subplots(1, 3, figsize=(10, 6))
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    axes[0].scatter(df[ee_columns[0]][start_ind:end_ind], df[ee_columns[1]][start_ind:end_ind], c=t, cmap='viridis', s=10, zorder=3)
    # axes[0].plot(df[ee_columns[0]][start_ind:end_ind], df[ee_columns[1]][start_ind:end_ind])
    # axes[0].add_collection(lc)
    # axes[0].plot(df['time'][start_ind:end_ind], df[goal_pos_columns[0]][start_ind:end_ind], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[0].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    # axes[0].set_xlim([0.3, 0.42])
    # axes[0].set_ylim([-0.35, 0.35])
    axes[0].set_ylim([-0.42, 0.42])
    axes[0].set_ylim([-0.42, 0.05])
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('X-Y Trajectory, Method')
    # axes[0].set_title('X-Y Trajectory, Baseline')
    # axes[0].legend(loc='lower left')
    axes[0].grid(True, zorder=0)  

    # Y
    axes[1].grid()   
    axes[1].scatter(df[ee_columns[1]][start_ind:end_ind], df[ee_columns[2]][start_ind:end_ind], c=t, cmap='viridis', s=10, zorder=3)
    # axes[1].plot(df[goal_pos_columns[1]][start_ind:end_ind], df[goal_pos_columns[1]][start_ind:end_ind], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[1].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    # axes[1].set_xlim([-0.35, 0.35])
    # axes[1].set_ylim([0.3, 0.38])
    axes[1].set_ylim([-0.42, 0.05])
    axes[1].set_ylim([0.15, 0.4])
    axes[1].set_xlabel('Y (m)')
    axes[1].set_ylabel('Z (m)')
    axes[1].set_title('Y-Z Trajectory, Method')
    # axes[1].set_title('Y-Z Trajectory, Baseline')
    axes[1].grid(True, zorder=0)  
    # axes[1].legend(loc='lower left')
    # axes[1].grid()   

    # Z
    sc = axes[2].scatter(df[ee_columns[0]][start_ind:end_ind], df[ee_columns[2]][start_ind:end_ind], c=t, cmap='viridis', s=10, zorder=3)
    # axes[2].plot(df[goal_pos_columns[2]][start_ind:end_ind], df[goal_pos_columns[2]][start_ind:end_ind], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[2].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    # axes[2].set_xlim([0.32, 0.4])
    # axes[2].set_ylim([0.3, 0.38])
    axes[2].set_xlim([-0.42, 0.42])
    axes[2].set_ylim([0.15, 0.4])
    axes[2].set_xlabel('X (m)')
    axes[2].set_ylabel('Z (m)')
    axes[2].set_title('X-Z Trajectory, Method')
    # axes[2].set_title('X-Z Trajectory, Baseline')
    # axes[2].legend(loc='lower left')
    axes[2].grid(True, zorder=0)  

    # Add colorbar
    # Add colorbar to the figure and link it to the scatter
    fig.colorbar(sc, ax=axes[2], location='right', label='Normalized Time')

    plt.tight_layout()  # Adjust spacing to prevent overlap
    # plt.show() 
    
    # Plot (x-y) plane plot 
    x = df[ee_columns[0]][0:]
    y = df[ee_columns[1]][0:]
    z = df[ee_columns[2]][0:]
    t = np.arange(0, len(x))
    
    # Normalize t for color mapping
    t_normalized = (t - t.min()) / (t.max() - t.min())

    # Create a colormap
    cmap = plt.get_cmap("plasma")

    # Map normalized t values to colors
    colors = cmap(t_normalized)

    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot each point with the corresponding color
    for i in range(len(x) - 1):
        ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colors[i], linewidth=2)

    # Set labels
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    link_z_offset = 0.3

    # Define a single blueish-grey RGB color
    blueish_grey_rgb = (0.6, 0.65, 0.7, 0.5)  # adjust as needed

    # Create a colormap with just one color
    single_color_cmap = mcolors.ListedColormap([blueish_grey_rgb])

    # Add link 0
    vertices, faces = load_obj('link0.obj')
    ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2] - 0.03, triangles=faces, cmap=single_color_cmap)

    # Add mesh 
    vertices, faces = load_obj('link1.obj')
    ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2] + link_z_offset, triangles=faces, cmap=single_color_cmap)
    
    # Add second link (apply transformation matrix)
    R = np.array([ [0.707107, 0.707107, -3.67321e-06],
                    [2.59734e-06, 2.59735e-06, 1],
                    [0.707107, -0.707107, 4.89659e-12] ])
    t = np.array([0, 0, 0.333]) - np.array([0, 0, 0.333])
    
    vertices, faces = load_obj('link2.obj')
    # ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2], triangles=faces, cmap='viridis')
    
    xyz = np.array([vertices[:, 0], vertices[:, 1], vertices[:, 2]]).T
    for i in range(np.shape(xyz)[0]):
        xyz[i, :] = R @ xyz[i, :] + t
    
    ax.plot_trisurf(xyz[:, 0], xyz[:, 1], xyz[:, 2] + link_z_offset, triangles=faces, cmap=single_color_cmap)
    
    # Plot hand 
    vertices, faces = load_obj('hand.obj')
    ax.plot_trisurf(x[0] + vertices[:, 0], y[0] + vertices[:, 1], z[0] + vertices[:, 2], triangles=faces, cmap=single_color_cmap)
    
    # Equalize x and y axis spacing
    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()

    # Set the same scale for both axes
    scale = max(x_max - x_min, y_max - y_min)
    # ax.set_xlim([x_min, x_min + scale])
    # ax.set_ylim([y_min, y_min + scale])

    # ax.set_aspect('equal', adjustable='box')
    
    # Set the view angle to top-down (looking along the z-axis)
    # ax.view_init(elev=90, azim=0)

    # Add color bar
    # cb = plt.colorbar(lc, ax=ax, orientation="vertical", label="Older → Newer")
    
    # ax.set_title('Top View (X-Y Plane) Self-Collision Avoidance, Baseline')
    # ax.set_title('Self-Collision Avoidance - Baseline')
    # ax.set_title('Top View (X-Y Plane) Self-Collision Avoidance, Method')
    ax.set_title('Self-Collision Avoidance - Method')
    
    fig.tight_layout()
    # Remove whitespace
    # fig.subplots_adjust(left=0, right=1, bottom=0, top=0)


    # Callback function
    def on_click(event):
        if event.inaxes == ax:
            elev = ax.elev
            azim = ax.azim
            print(f"Mouse clicked. Elevation: {elev}, Azimuth: {azim}")

    # Connect the event
    cid = fig.canvas.mpl_connect('button_press_event', on_click)

    # elev = 44.544037412314864
    # azim = 30.724863600935223
    # elev = 25.604053000779416
    # azim = 33.67108339828518
    # elev = 21.18158234548224
    # azim = 25.630227661381294
    # elev = 30.227545049499156
    # azim = 32.66597643117226
    elev = 36.01865962471349
    azim = 45.29263270552924

    # Mouse clicked. Elevation: 30.227545049499156, Azimuth: 32.66597643117226
    # Mouse clicked. Elevation: 33.17376484684912, Azimuth: 45.29263270552924
    # Mouse clicked. Elevation: 41.59153569642043, Azimuth: 23.827317039122384


    ax.view_init(elev=elev, azim=azim)  # 30° elevation, 45° azimuth

    # Apply equal ratio
    # equal_ratio(ax)

    # Get and print axis limits
    # ax.set_xlim([-0.4682776145833334, 0.4921106145833334])
    # ax.set_ylim([-0.43780003164809744, 0.5225881975185693])
    # ax.set_zlim([-0.18816424283233008, 0.7722239863343368])

    ax.set_xlim([-0.5, 0.2])
    ax.set_ylim([-0.6, 0.2])
    ax.set_zlim([-0.2, 0.4])

    # xlim = ax.get_xlim3d()
    # ylim = ax.get_ylim3d()
    # zlim = ax.get_zlim3d()

    # print(f"X axis limits: {xlim}")
    # print(f"Y axis limits: {ylim}")
    # print(f"Z axis limits: {zlim}")

    ax.set_aspect('equal', adjustable='box')

    plt.show()
    

# Example usage
# plot_joint("../../examples/97-self_collision/joints.csv")
# plot_joint("../../examples/97-self_collision/data/06-04/joints.csv")  # paper data

# plot_joint("../../examples/97-self_collision/data/06-11-high-velocity/joints-baseline.csv")
# plot_joint("../../examples/97-self_collision/data/06-11-high-velocity/joints-method-matching.csv")

plot_joint("../../examples/97-self_collision/data/06-18-revised/joints-FINAL.csv")
# plot_joint("../../examples/97-self_collision/data/06-18-revised/joints.csv")
# plot_joint("../../build/examples/97-self_collision/joints.csv")

