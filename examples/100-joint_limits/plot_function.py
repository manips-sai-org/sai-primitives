import numpy as np
import matplotlib.pyplot as plt

# Define constants
a = 2
b = 5

# Define the function
def smooth_step(x, a, b):
    # return a + (b - a) * (1 - np.cos(np.pi * x)) / 2
    return b + (a - b) * (1 - np.cos(np.pi * x)) / 2  # entry + (exit - entry)

# Generate x values
x = np.linspace(0, 1, 500)
y = smooth_step(x, a, b)

# Plot
# plt.figure(figsize=(8, 4))
# plt.figure(figsize=(4, 4))
plt.plot(x, y, label=r'$f(x) = a + (b - a) \cdot \frac{1 - \cos(\pi x)}{2}$')
plt.title('Maximum Velocity Saturation Function')
plt.xlabel(r'Normalized Distance ($\alpha$)')
plt.ylabel('Velocity')
# plt.legend()

# Add unlabeled intermediate y-ticks to get more horizontal grid lines
num_grid_lines = 6
yticks = np.linspace(a, b, num_grid_lines)
ytick_labels = ['' for _ in yticks]
ytick_labels[0] = '$V_{exit}$'
ytick_labels[-1] = '$V_{entry}$'
plt.yticks(yticks, ytick_labels)

# Add grid
plt.grid(True, which='both', axis='both', linestyle='--', alpha=0.5)

plt.tight_layout()
plt.show()
