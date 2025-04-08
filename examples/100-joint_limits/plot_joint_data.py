import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
matplotlib.use('TkAgg')  # Alternative: 'Qt5Agg'

# Joint limits
q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
# offset = 6 * np.pi / 180 
# lower_offset = 9 * np.pi / 180 
offset = 4 * np.pi / 180 
lower_offset = 6 * np.pi / 180 
dq_max = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61] 

# Load CSV file

def plot_joint(csv_file, joint_index):
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

    # Extrat apf torques
    apf_columns = [col for col in df.columns if 'apf_torques__' in col]

    # Ensure joint_index is within range
    if joint_index < 0 or joint_index >= len(joint_columns):
        raise ValueError("Invalid joint index")   

    joint = joint_columns[joint_index]   

    """
        Plot selected joint (all data)
    """
    # Plot the selected joint
    # plt.figure(figsize=(20, 10))
    # plt.figure()
    fig, axes = plt.subplots(4, 1, figsize=(10, 10))
    # plt.subplot(1, 1, 1)
    axes[0].plot(df['time'], df[joint], label='Joint ' + str(joint_index))

    # Draw horizontal dashed lines
    axes[0].axhline(y=q_min[joint_index] + lower_offset, color='g', linestyle='--')
    axes[0].axhline(y=q_max[joint_index] - lower_offset, color='g', linestyle='--')
    axes[0].axhline(y=q_min[joint_index] + offset, color='orange', linestyle='--')
    axes[0].axhline(y=q_max[joint_index] - offset, color='orange', linestyle='--')
    axes[0].axhline(y=q_min[joint_index], color='r', linestyle='--')
    axes[0].axhline(y=q_max[joint_index], color='r', linestyle='--')   

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[joint_pos_state_columns[joint_index]][i] != 0:
            axes[0].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.02)

    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Joint Angle (rad)')
    axes[0].set_title(f'Robot Joint {joint_index + 1} Angle Over Time')
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
    axes[1].set_title(f'Robot Joint {joint_index + 1} Velocity Over Time')
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
    axes[2].set_title(f'Robot Joint {joint_index + 1} Torque Over Time')
    axes[2].legend()
    axes[2].grid()   

    # APF Torques
    axes[3].plot(df['time'], df[apf_columns[joint_index]], label=joint)   

    # Draw horizontal dashed lines
    # plt.axhline(y=dq_max[joint_index] + offset, color='g', linestyle='--')
    # plt.axhline(y=dq_max[joint_index] - offset, color='g', linestyle='--')
    # plt.axhline(y=dq_max[joint_index], color='r', linestyle='--')
    # plt.axhline(y=-dq_max[joint_index], color='r', linestyle='--')  
    axes[3].set_xlabel('Time (s)')
    axes[3].set_ylabel('Joint Torque (N-m)')
    axes[3].set_title(f'Robot Joint {joint_index + 1} Torque Over Time')
    axes[3].legend()
    axes[3].grid()   
    
    plt.tight_layout()  # Adjust spacing to prevent overlap
    # plt.show() 

    """
        Plot selected joint (only position)
    """
    start = 4  # Start value
    end = 19   # End value
    dt = 0.01  # Interval (100 Hz)
    num_points = int((end - start) / dt) + 1  # Calculate number of points
    time_indices = np.arange(500, 2100, 1)
    zoom_time_indices = np.arange(500, 1050, 1)

    # Plot the selected joint
    # plt.figure(figsize=(20, 10))
    # plt.figure()
    # fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    fig, axes = plt.subplots(1, 1)
    # plt.subplot(1, 1, 1)
    axes.plot(df['time'][time_indices], df[joint][time_indices], label='Joint ' + str(joint_index + 1))

    # Draw horizontal dashed lines
    axes.axhline(y=q_min[joint_index] + lower_offset, color='g', linestyle='--')
    axes.axhline(y=q_max[joint_index] - lower_offset, color='g', linestyle='--')
    axes.axhline(y=q_min[joint_index] + offset, color='orange', linestyle='--')
    axes.axhline(y=q_max[joint_index] - offset, color='orange', linestyle='--')
    axes.axhline(y=q_min[joint_index], color='r', linestyle='--')
    axes.axhline(y=q_max[joint_index], color='r', linestyle='--')   

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    for i in range(len(df['time'])):
        if df[joint_pos_state_columns[joint_index]][i] != 0:
            axes.axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=2, alpha=0.05)

    axes.set_xlabel('Time (s)')
    axes.set_ylabel('Joint Angle (rad)')
    axes.set_title(f'Robot Joint {joint_index + 1} Angle Over Time')

    # axes.set_xlim([5, 11])
    # axes.set_ylim([-3.2, -2.0])

    axes.legend(loc="upper left")  # Outside top-right corner
    axes.grid()

    plt.tight_layout()  # Adjust spacing to prevent overlap
    # plt.show() 


# Example usage
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 2)
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 3)
# plot_joint("./data/03-21/joints_baseline.csv", 3)
plot_joint("./data/03-21/joints.csv", 3)
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 5)
# plot_joint("./data/joints.csv", 3)
plt.show()