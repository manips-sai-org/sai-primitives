import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from matplotlib.collections import LineCollection
matplotlib.use('TkAgg')  # Alternative: 'Qt5Agg'

# Joint limits
q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
offset = (8 - 4) * np.pi / 180 
vel_offset = (12 - 4) * np.pi / 180
# offset = 4 * np.pi / 180 
# lower_offset = 9 * np.pi / 180 
dq_max = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61] 

# Virtual limits 
# q_min[2] = -0.3
# q_max[2] = 0.3

q_max[3] = -1.89182 + 0.3

q_vel_limit = [-2, -2, -2]

# Load CSV file
def plot_data(csv_file, joint_index=3):
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

    # Get upper offset and lower offset for the plots 

    # """ 
    #     Plot joint data (all)
    # """

    # # Plot joint data 
    # # Plot the selected joint
    # fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    # # plt.subplot(1, 1, 1)
    # axes[0].plot(df['time'], df[joint], label=joint)   

    # # Draw horizontal dashed lines
    # axes[0].axhline(y=q_min[joint_index] + lower_offset, color='g', linestyle='--')
    # axes[0].axhline(y=q_max[joint_index] - lower_offset, color='g', linestyle='--')
    # axes[0].axhline(y=q_min[joint_index] + offset, color='y', linestyle='--')
    # axes[0].axhline(y=q_max[joint_index] - offset, color='y', linestyle='--')
    # axes[0].axhline(y=q_min[joint_index], color='r', linestyle='--')
    # axes[0].axhline(y=q_max[joint_index], color='r', linestyle='--')   

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[joint_pos_state_columns[joint_index]][i] != 0:
    #         axes[0].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.005)

    # axes[0].set_xlabel('Time (s)')
    # axes[0].set_ylabel('Joint Angle (rad)')
    # axes[0].set_title(f'Robot Joint {joint_index} Angle Over Time')
    # axes[0].legend()
    # axes[0].grid()   

    # # Velocity
    # # plt.figure(figsize=(10, 6))
    # # plt.subplot(2, 1, 1)
    # axes[1].plot(df['time'], df[joint_vel_columns[joint_index]], label=joint)   

    # # Draw horizontal dashed lines
    # # plt.axhline(y=dq_max[joint_index] + offset, color='g', linestyle='--')
    # # plt.axhline(y=dq_max[joint_index] - offset, color='g', linestyle='--')
    # axes[1].axhline(y=dq_max[joint_index], color='r', linestyle='--')
    # axes[1].axhline(y=-dq_max[joint_index], color='r', linestyle='--')  
    # axes[1].set_xlabel('Time (s)')
    # axes[1].set_ylabel('Joint Velocity (rad/s)')
    # axes[1].set_title(f'Robot Joint {joint_index} Velocity Over Time')
    # axes[1].legend()
    # axes[1].grid()   

    # # Torque
    # # plt.subplot(3, 1, 1)
    # # plt.figure(figsize=(10, 6))
    # axes[2].plot(df['time'], df[joint_torque_columns[joint_index]], label=joint)   

    # # Draw horizontal dashed lines
    # # plt.axhline(y=dq_max[joint_index] + offset, color='g', linestyle='--')
    # # plt.axhline(y=dq_max[joint_index] - offset, color='g', linestyle='--')
    # # plt.axhline(y=dq_max[joint_index], color='r', linestyle='--')
    # # plt.axhline(y=-dq_max[joint_index], color='r', linestyle='--')  
    # axes[2].set_xlabel('Time (s)')
    # axes[2].set_ylabel('Joint Torque (N-m)')
    # axes[2].set_title(f'Robot Joint {joint_index} Torque Over Time')
    # axes[2].legend()
    # axes[2].grid()  

    # plt.tight_layout()  # Adjust spacing to prevent overlap

    """ 
        Plot joint data (only position)
    """

    # Baseline 
    # start_time = 6.8 
    # end_time = 20 

    start_time = 7 - 2
    # end_time = 15 - 1
    end_time = 10

    # Method
    # start_time = 7
    # end_time = 15
    # start_time = 7.5
    # end_time = 21.3

    # # Method (fast)
    # start_time = 7.5
    # end_time = 20.7

    # Plot joint data 
    # Plot the selected joint
    # fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    fig, axes = plt.subplots(1, 1)
    # time_index = np.arange(700, 2060, 1)  # method 
    # time_index = np.arange(700, 2030, 1)  # baseline 
    time_index = np.arange(int(start_time * 100), int(end_time * 100), 1)
    # time_index = np.arange(0, len(df['time']), 1)
    # : = np.arange(0, np.shape(q_min)[0])
    # plt.subplot(1, 1, 1)
    axes.plot(df['time'][time_index] - start_time, df[joint][time_index], label='Joint 4')   

    # Draw horizontal dashed lines
    # axes.axhline(y=-2.05, color='orange', linestyle='--')
    # axes.axhline(y=-2.0, color='orange', linestyle='--')
    # axes.axhline(y=-2.1, color='orange', linestyle='--')
    axes.axhline(y=q_max[joint_index] - vel_offset, color='orange', linestyle='--')
    # axes.axhline(y=0.1, color='orange', linestyle='--')
    # axes.axhline(y=q_min[joint_index] + offset, color='red', linestyle='--')
    axes.axhline(y=q_max[joint_index] - offset, color='red', linestyle='--')
    # axes.axhline(y=q_min[joint_index], color='black', linestyle='--')
    axes.axhline(y=q_max[joint_index], color='black', linestyle='--')   

    # # Draw pos and vel constraint lines
    # Plot vertical yellow bars
    # for i in range(len(df['time'])):
        # if df[joint_pos_state_columns[joint_index]][i] != 0:
            # if (df['time'][i] > 8.3):
                # axes.axvline(x=df['time'][i] - start_time, color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 5)

    # axes.set_xlim([7.5, 21])  # overall plot
    # axes.set_xlim([0, end_time - start_time])  # baseline, overall
    # axes.set_ylim([-0.33, 0.33])

    # axes.set_xlim([14.5, 20.6])  # zoomed plot
    # axes.set_xlim([14.5, 20.6])  # zoomed plot, baseline 

    # axes.set_ylim([-0.35, 0])  # zoomed plot
    # axes.set_ylim([])

    axes.set_xlim([0, end_time - start_time])
    axes.set_ylim([-2.4, -1.5])

    axes.set_xlabel('Time (s)')
    axes.set_ylabel('Joint Angle (rad)')
    axes.set_title(f'Joint {joint_index+1} Angle, Normal')
    # axes.set_title(f'Joint {joint_index+1} Angle, Baseline')
    # axes.set_title(f'Joint {joint_index+1} Angle, Method')
    # axes.set_title(f'Joint {joint_index+1} Angle, Method (Tight Bounds)')
    # axes.set_title(f'Joint {joint_index+1} Angle, Method (Fast Velocity + Tight Bounds)')
    # axes.legend()
    axes.grid()   

    plt.tight_layout()  # Adjust spacing to prevent overlap

    """ 
        Plot ee information
    """

    # Plot ee position 
    # fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    # fig, axes = plt.subplots(1, 3, figsize=(10, 6))
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    axes[0].plot(df['time'][time_index], df[ee_pos_columns[0]][time_index], label='Current', color='b')
    axes[0].plot(df['time'][time_index], df[goal_pos_columns[0]][time_index], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[0].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    axes[0].set_xlim([7, 20.6])
    axes[0].set_ylim([0.1, 0.6])
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('X (m)')
    axes[0].set_title('X Trajectory')
    axes[0].legend(loc='lower left')
    axes[0].grid()   

    # Y
    axes[1].plot(df['time'][time_index], df[ee_pos_columns[1]][time_index], label='Current', color='b')
    axes[1].plot(df['time'][time_index], df[goal_pos_columns[1]][time_index], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[1].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    axes[1].set_xlim([7, 20.6])
    axes[1].set_ylim([-0.5, 0.5])
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Y (m)')
    axes[1].set_title('Y Trajectory')
    axes[1].legend(loc='lower left')
    axes[1].grid()   

    # Z
    axes[2].plot(df['time'][time_index], df[ee_pos_columns[2]][time_index], label='Current', linestyle='-', color='b')
    axes[2].plot(df['time'][time_index], df[goal_pos_columns[2]][time_index], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[2].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    axes[2].set_xlim([7, 20.6])
    axes[2].set_ylim([0.1, 0.6])
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Z (m)')
    axes[2].set_title('Z Trajectory')
    axes[2].legend(loc='lower left')
    axes[2].grid()   

    plt.tight_layout()  # Adjust spacing to prevent overlap
    # plt.show() 

    """ 
        EE spatial plot (x vs y), (y vs. z), (x vs. z)
    """
    # Example: assume df, ee_pos_columns, and time_index are defined
    x = df[ee_pos_columns[1]][time_index].values
    y = df[ee_pos_columns[2]][time_index].values

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
    sc = axes[0].scatter(df[ee_pos_columns[0]][time_index], df[ee_pos_columns[1]][time_index], c=t, cmap='viridis', s=10, zorder=3)
    # axes[0].plot(df[ee_pos_columns[0]][time_index], df[ee_pos_columns[1]][time_index])
    # axes[0].add_collection(lc)
    # axes[0].plot(df['time'][time_index], df[goal_pos_columns[0]][time_index], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[0].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    axes[0].set_xlim([0.3, 0.42])
    axes[0].set_ylim([-0.35, 0.35])
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('X-Y Trajectory, Method')
    # axes[0].legend(loc='lower left')
    axes[0].grid()   

    # Y
    axes[1].scatter(df[ee_pos_columns[1]][time_index], df[ee_pos_columns[2]][time_index], c=t, cmap='viridis', s=10, zorder=3)
    # axes[1].plot(df[goal_pos_columns[1]][time_index], df[goal_pos_columns[1]][time_index], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[1].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    axes[1].set_xlim([-0.35, 0.35])
    axes[1].set_ylim([0.3, 0.38])
    axes[1].set_xlabel('Y (m)')
    axes[1].set_ylabel('Z (m)')
    axes[1].set_title('Y-Z Trajectory, Method')
    # axes[1].legend(loc='lower left')
    axes[1].grid()   

    # Z
    axes[2].scatter(df[ee_pos_columns[0]][time_index], df[ee_pos_columns[2]][time_index], c=t, cmap='viridis', s=10, zorder=3)
    # axes[2].plot(df[goal_pos_columns[2]][time_index], df[goal_pos_columns[2]][time_index], label='Goal', linestyle='--', color='g')

    # # # Draw pos and vel constraint lines
    # # Plot vertical yellow bars
    # for i in range(len(df['time'])):
    #     if df[' locked_joint'][i] != 0:
    #         axes[2].axvline(x=df['time'][i], color='yellow', linestyle='-', linewidth=5, alpha=0.007 * 1)

    axes[2].set_xlim([0.32, 0.4])
    axes[2].set_ylim([0.3, 0.38])
    axes[2].set_xlabel('X (m)')
    axes[2].set_ylabel('Z (m)')
    axes[2].set_title('X-Z Trajectory, Method')
    # axes[2].legend(loc='lower left')
    axes[2].grid()   

    # Add a shared colorbar to the right of the entire figure
    fig.colorbar(sc, ax=axes[2], location='right', label='Normalized Time')

    plt.tight_layout()  # Adjust spacing to prevent overlap
    # plt.show() 
 

# Example usage
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 2)
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 3)
# plot_joint("../../build/examples/100-joint_limits/joints.csv", 5)

# plot_data("../../build/examples/100-joint_limits/virtual.csv")

# plot_data("./data/06-12-virtual-limits/virtual-baseline-final.csv")
# plot_data("./data/06-12-virtual-limits/virtual-method-backup.csv")

# plot_data("./data/06-11-virtual-limits/virtual-baseline.csv")

# plot_data("./data/06-11-virtual-limits/virtual-method.csv")
# plot_data("./data/06-11-virtual-limits/virtual-method.csv")

# Final paper data 
# plot_data("./data/06-13-virtual-limits/virtual-baseline.csv")
# plot_data("./data/06-13-virtual-limits/virtual-method.csv")
# plot_data("./data/06-13-virtual-limits/virtual-method-1.4.csv")

# Final ISER presentation data 
# plot_data("./data/06-20-virtual-limits-3/virtual-baseline.csv")
# plot_data("./data/06-20-virtual-limits-3/virtual-method.csv")
# plot_data("./data/06-20-virtual-limits-3/virtual-method-tight.csv")
# plot_data("./data/06-20-virtual-limits-3/virtual-method-tight-fast.csv")

# Final ISER rev2
# plot_data("./data/07-03-virtual-limits/virtual-baseline.csv")
# plot_data("./data/07-03-virtual-limits/virtual-method-12_8.csv")
# plot_data("./data/07-03-virtual-limits/virtual-method-8_4.csv")
plot_data("./data/07-03-virtual-limits/virtual-normal.csv")

plt.show()