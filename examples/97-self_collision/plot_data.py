import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
matplotlib.use('TkAgg')  # Alternative: 'Qt5Agg'

# Joint limits
q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
offset = 6 * np.pi / 180 
dq_max = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61] 

# Load CSV file

def plot_data(csv_file):
    df = pd.read_csv(csv_file)   

    # Drop 'timestamp' column if it exists
    if 'timestamp' in df.columns:
        df = df.drop(columns=['timestamp'])  

    # Convert 'time' column to float
    df['time'] = df['time'].astype(float)  

    # Extract data columns
    ee_pos_columns = [col for col in df.columns if 'ee_pos__' in col]   
    goal_pos_columns = [col for col in df.columns if 'goal_pos__' in col]   
    joint_columns = [col for col in df.columns if 'robot_q__' in col]
    collision_state_columns = [col for col in df.columns if 'collision_state__' in col]  
    collision_distance_columns = [col for col in df.columns if 'collision_distance__' in col] 
    collision_closest_point_columns = [col for col in df.columns if 'closest_point_in_world__' in col] 

    # Data
    closest_distance = df[' collision_closest_distance']
    constraint_flag = df[' constraint_flag']

    # Plot ee position 
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    axes[0].plot(df['time'], df[ee_pos_columns[0]], label='x current', color='b')
    axes[0].plot(df['time'], df[goal_pos_columns[0]], label='x goal', linestyle='--', color='g')

    # Draw horizontal dashed lines
    axes[0].plot(df['time'], df[collision_closest_point_columns[0]], color='r', linestyle='--', label='x closest')

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[' constraint_flag'][i] != 0:
            axes[0].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.005)

    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('X (m)')
    axes[0].set_title('X Trajectory')
    axes[0].legend()
    axes[0].grid()   

    # Y
    axes[1].plot(df['time'], df[ee_pos_columns[1]], label='y current', color='b')
    axes[1].plot(df['time'], df[goal_pos_columns[1]], label='y goal', color='g')

    # Draw horizontal dashed lines
    axes[1].plot(df['time'], df[collision_closest_point_columns[1]], color='r', linestyle='--', label='y closest')

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[' constraint_flag'][i] != 0:
            axes[1].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.005)

    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Y (m)')
    axes[1].set_title('Y Trajectory')
    axes[1].legend()
    axes[1].grid()   

    # Z
    axes[2].plot(df['time'], df[ee_pos_columns[2]], label='z current', color='b')
    axes[2].plot(df['time'], df[goal_pos_columns[2]], label='z goal', color='g')

    # Draw horizontal dashed lines
    axes[2].plot(df['time'], df[collision_closest_point_columns[2]], color='r', linestyle='--', label='z closest')

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[' constraint_flag'][i] != 0:
            axes[2].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.005)

    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Z (m)')
    axes[2].set_title('Z Trajectory')
    axes[2].legend()
    axes[2].grid()   

    plt.tight_layout()  # Adjust spacing to prevent overlap
    # plt.show() 

    

# Example usage
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 2)
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 3)
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 5)
# plot_data("../../build/examples/97-self_collision/joints.csv")
plot_data("./data/joints.csv")
plt.show()