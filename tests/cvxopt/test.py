import cvxpy as cp
import numpy as np

def solve_sdp_relaxation(H_list, V):
    # 1. Precompute A_i = V^T H_i V
    A_list = [V.T @ H @ V for H in H_list]
    k = A_list[0].shape[0]

    # 2. Define the Variable X (the relaxation of xx^T)
    X = cp.Variable((k, k), symmetric=True)

    # 3. Define the Objective
    # Since (Tr(A_i X))^2 is convex (it's a power of a linear function), 
    # we can use cp.quad_over_lin or simply cp.sum_squares of the traces.
    traces = [cp.trace(A @ X) for A in A_list]
    objective = cp.Minimize(cp.sum_squares(cp.vstack(traces)))

    # 4. Define Constraints
    constraints = [
        X >> 0,           # X is Positive Semidefinite
        cp.trace(X) == 1  # Trace(X) = 1 is the relaxation of ||x||=1
    ]

    # 5. Solve
    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.SCS) # Or cp.MOSEK / cp.CVXOPT

    # 6. "Rounding" to get x
    # Extract the principal eigenvector of the resulting matrix X
    vals, vecs = np.linalg.eigh(X.value)
    x_opt = vecs[:, -1] # Highest eigenvalue eigenvector
    
    return x_opt, prob.value

# --- Usage ---
n, k = 10, 5
H_list = [np.random.randn(n, n) for _ in range(3)]
H_list = [(h + h.T)/2 for h in H_list]
V = np.linalg.qr(np.random.randn(n, k))[0]

x_res, val = solve_sdp_relaxation(H_list, V)
print(f"Approximated Optimal x: {x_res}")