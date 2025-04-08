import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
matplotlib.use('TkAgg')  # Alternative: 'Qt5Agg'


"""
    Plot singularity data:
    1.)  current vs. goal position and highlight 
    2.)  orientation error and highlight 
    3.)  s-value ratios and highlight w/ bars 
    4.)  Singular vs. non-singular task torques (single joint) + highlight 

    Data: 	
    logger.addToLog(robot_q, "robot_q");
	logger.addToLog(robot_dq, "robot_dq");
	logger.addToLog(ee_pos, "ee_pos");
	logger.addToLog(goal_pos, "goal_pos");
	logger.addToLog(svalues, "svalues");
    logger.addToLog(singular_task_torques, "singular_task_torques");
	logger.addToLog(motion_force_task_torques, "motion_task_torques");
	logger.addToLog(alpha, "alpha");
	logger.addToLog(ori_error, "ori_error");
    logger.addToLog(singular_direction, "singular_direction");
"""

# plot options 
# TYPE_1_OPT = True 
# TYPE_2_OVERHEAD_OPT = False 
# TYPE_2_WRIST_OPT = False 
PLOT_OPT = [1, 0, 0]
YLABELS = ["X (m)", "Y (m)", "Z (m)"]
ORI_YLABELS = ["X (rad)", "Y (rad)", "Z (rad)"]
LABELS = ["Type 1 Singularity", "Type 2 Singularity", "Type 2 Singularity"]

# s values 
combined_s_values = [6e-3, 6e-2]
linear_s_values = [3e-2, 3e-1]

# Load CSV file
def plot(csv_file, joint_index, exp_index):
    df = pd.read_csv(csv_file)   

    # Drop 'timestamp' column if it exists
    if 'timestamp' in df.columns:
        df = df.drop(columns=['timestamp'])  

    # Convert 'time' column to float
    df['time'] = df['time'].astype(float)  

    # Extract data columns 
    ee_pos_columns = [col for col in df.columns if 'ee_pos' in col]  
    goal_pos_columns = [col for col in df.columns if 'goal_pos' in col]
    ori_error_columns = [col for col in df.columns if 'ori_error' in col]
    svalue_columns = [col for col in df.columns if 'svalues' in col]
    singular_task_torques_columns = [col for col in df.columns if 'singular_task_torques' in col]
    motion_task_torques_columns = [col for col in df.columns if 'motion' in col]
    alpha_columns = [col for col in df.columns if 'alpha' in col]

    # Get indices for when alpha != 1
    s_indices = np.where(np.array(df[alpha_columns]) != 1)[0]

    """
        Plot data 
    """

    # Plot ee vs. goal position 
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    for i in range(3):
        axes[i].plot(df['time'], df[ee_pos_columns[i]], color='b')
        axes[i].plot(df['time'], df[goal_pos_columns[i]], color='g', linestyle='--')

        # Plot vertical yellow bars
        # for j in range(len(s_indices)):
            # if s_indices[j] != 0:
        for j in s_indices:
            axes[i].axvline(x=df['time'][j], color='yellow', linestyle='-', linewidth=5, alpha=0.05)

        axes[i].set_xlabel('Time (s)')
        axes[i].set_ylabel(YLABELS[i])
        axes[i].set_title(LABELS[exp_index])
        axes[i].legend()
        axes[i].grid() 

    plt.tight_layout()

    # Plot orientation error 
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    for i in range(3):
        axes[i].plot(df['time'], df[ori_error_columns[i]], color='b')

        # Plot vertical yellow bars
        # for j in range(len(s_indices)):
            # if s_indices[j] != 0:
            # axes[i].axvline(x=df['time'][s_indices[j]], color='yellow', linestyle='-', linewidth=5, alpha=0.05)
        for j in s_indices:
            axes[i].axvline(x=df['time'][j], color='yellow', linestyle='-', linewidth=5, alpha=0.05)

        axes[i].set_xlabel('Time (s)')
        axes[i].set_ylabel(ORI_YLABELS[i])
        axes[i].set_title(LABELS[exp_index])
        axes[i].legend()
        axes[i].grid() 

    plt.tight_layout()

    # Plot s value ratios
    svalue_ratios = df[svalue_columns].to_numpy(dtype=np.float64)
    normalized_matrix = svalue_ratios / svalue_ratios.max(axis=1, keepdims=True)
    fig, axes = plt.subplots(1, 1)
    axes.plot(df['time'], normalized_matrix[:, 0], label="S1")
    axes.plot(df['time'], normalized_matrix[:, 1], label="S2")
    axes.plot(df['time'], normalized_matrix[:, 2], label="S3")
    axes.plot(df['time'], normalized_matrix[:, 3], label="S4")
    axes.plot(df['time'], normalized_matrix[:, 4], label="S5")
    axes.plot(df['time'], normalized_matrix[:, 5], label="S6")

    # Draw horizontal dashed lines
    axes.axhline(y=linear_s_values[0], color='red', linestyle='--')
    axes.axhline(y=linear_s_values[1], color='orange', linestyle='--')

    axes.set_xlabel('Time (s)')
    axes.set_ylabel('Magnitude')
    axes.set_title('Singular Value Ratios')
    # axes.legend()
    axes.grid() 

    plt.tight_layout()

    # Plot singular vs. strategy torques for a joint 
    singular_joint_torque = df[singular_task_torques_columns[joint_index]]
    motion_force_joint_torque = df[motion_task_torques_columns[joint_index]]
    fig, axes = plt.subplots(1, 1)
    axes.plot(df['time'], singular_joint_torque)
    axes.plot(df['time'], motion_force_joint_torque)

     # Plot vertical yellow bars
    # for j in range(len(s_indices)):
        # if s_indices[j] != 0:
        # axes.axvline(x=df['time'][s_indices[j]], color='yellow', linestyle='-', linewidth=5, alpha=0.05)
    for j in s_indices:
        axes.axvline(x=df['time'][j], color='yellow', linestyle='-', linewidth=5, alpha=0.05)

    axes.set_xlabel('Time (s)')
    axes.set_ylabel('Torque (N-m)')
    axes.set_title('Joint Torque With And Without Strategy')
    axes.legend()
    axes.grid() 
   
    plt.tight_layout()  # Adjust spacing to prevent overlap
    # plt.show() 

# Example usage
# plot("./data/type_2_wrist.csv", 0, 0)
plot("../../build/examples/18-panda_singularity/type_1.csv", 0, 0)
# plot("../../build/examples/18-panda_singularity/type_2_overhead.csv", 0, 0)
# plot("../../build/examples/18-panda_singularity/type_2_wrist.csv", 0, 0)
plt.show()