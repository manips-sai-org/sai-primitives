import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
matplotlib.use('TkAgg')  # Alternative: 'Qt5Agg'

# Joint limits
q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
offset = 6 * np.pi / 180 
lower_offset = 9 * np.pi / 180 
dq_max = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61] 

# Virtual limits 
q_min[2] = -0.2
q_max[2] = 0.2

# Load CSV file
def plot_data(csv_file, joint_index=2):
    df = pd.read_csv(csv_file)   

    # Drop 'timestamp' column if it exists
    if 'timestamp' in df.columns:
        df = df.drop(columns=['timestamp'])  

    # Convert 'time' column to float
    df['time'] = df['time'].astype(float)  

    # Extract joint columns
    joint_columns = [col for col in df.columns if 'robot_q__' in col]  

    # Extract velocity columns
    joint_vel_columns = [col for col in df.columns if 'robot_dq__' in col]  

    # Extract torque
    joint_torque_columns = [col for col in df.columns if 'robot_torque__' in col]   

    # Extract joint states
    joint_pos_state_columns = [col for col in df.columns if 'joint_pos_state__' in col]
    joint_vel_state_columns = [col for col in df.columns if 'joint_vel_state__' in col]

    # Ensure joint_index is within range
    if joint_index < 0 or joint_index >= len(joint_columns):
        raise ValueError("Invalid joint index")   

    joint = joint_columns[joint_index]   

    # Extract data columns
    ee_pos_columns = [col for col in df.columns if 'ee_pos__' in col]   
    goal_pos_columns = [col for col in df.columns if 'goal_pos__' in col]   

    """ 
        Plot joint data
    """

    # Plot joint data 
    # Plot the selected joint
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    # plt.subplot(1, 1, 1)
    axes[0].plot(df['time'], df[joint], label=joint)   

    # Draw horizontal dashed lines
    axes[0].axhline(y=q_min[joint_index] + lower_offset, color='g', linestyle='--')
    axes[0].axhline(y=q_max[joint_index] - lower_offset, color='g', linestyle='--')
    axes[0].axhline(y=q_min[joint_index] + offset, color='y', linestyle='--')
    axes[0].axhline(y=q_max[joint_index] - offset, color='y', linestyle='--')
    axes[0].axhline(y=q_min[joint_index], color='r', linestyle='--')
    axes[0].axhline(y=q_max[joint_index], color='r', linestyle='--')   

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[joint_pos_state_columns[joint_index]][i] != 0:
            axes[0].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.005)

    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Joint Angle (rad)')
    axes[0].set_title(f'Robot Joint {joint_index} Angle Over Time')
    axes[0].legend()
    axes[0].grid()   

    # Velocity
    # plt.figure(figsize=(10, 6))
    # plt.subplot(2, 1, 1)
    axes[1].plot(df['time'], df[joint_vel_columns[joint_index]], label=joint)   

    # Draw horizontal dashed lines
    # plt.axhline(y=dq_max[joint_index] + offset, color='g', linestyle='--')
    # plt.axhline(y=dq_max[joint_index] - offset, color='g', linestyle='--')
    axes[1].axhline(y=dq_max[joint_index], color='r', linestyle='--')
    axes[1].axhline(y=-dq_max[joint_index], color='r', linestyle='--')  
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Joint Velocity (rad/s)')
    axes[1].set_title(f'Robot Joint {joint_index} Velocity Over Time')
    axes[1].legend()
    axes[1].grid()   

    # Torque
    # plt.subplot(3, 1, 1)
    # plt.figure(figsize=(10, 6))
    axes[2].plot(df['time'], df[joint_torque_columns[joint_index]], label=joint)   

    # Draw horizontal dashed lines
    # plt.axhline(y=dq_max[joint_index] + offset, color='g', linestyle='--')
    # plt.axhline(y=dq_max[joint_index] - offset, color='g', linestyle='--')
    # plt.axhline(y=dq_max[joint_index], color='r', linestyle='--')
    # plt.axhline(y=-dq_max[joint_index], color='r', linestyle='--')  
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Joint Torque (N-m)')
    axes[2].set_title(f'Robot Joint {joint_index} Torque Over Time')
    axes[2].legend()
    axes[2].grid()  

    plt.tight_layout()  # Adjust spacing to prevent overlap

    """ 
        Plot ee information
    """

    # Plot ee position 
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    axes[0].plot(df['time'], df[ee_pos_columns[0]], label='x current', color='b')
    axes[0].plot(df['time'], df[goal_pos_columns[0]], label='x goal', linestyle='--', color='g')

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[' locked_joint'][i] != 0:
            axes[0].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.005)

    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('X (m)')
    axes[0].set_title('X Trajectory')
    axes[0].legend()
    axes[0].grid()   

    # Y
    axes[1].plot(df['time'], df[ee_pos_columns[1]], label='y current', color='b')
    axes[1].plot(df['time'], df[goal_pos_columns[1]], label='y goal', color='g')

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[' locked_joint'][i] != 0:
            axes[1].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.005)

    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Y (m)')
    axes[1].set_title('Y Trajectory')
    axes[1].legend()
    axes[1].grid()   

    # Z
    axes[2].plot(df['time'], df[ee_pos_columns[2]], label='z current', color='b')
    axes[2].plot(df['time'], df[goal_pos_columns[2]], label='z goal', color='g')

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[' locked_joint'][i] != 0:
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
plot_data("../../build/examples/100-joint_limits/virtual.csv")
plt.show()