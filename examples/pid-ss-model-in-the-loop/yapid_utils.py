import numpy as np
from scipy.signal import cont2discrete


def print_cpp_matrix(name, mat, storage="static const float"):
    rows, cols = mat.shape
    print(f"{storage} {name}[{rows}][{cols}] = {{")
    for i in range(rows):
        row = ", ".join(f"{mat[i, j]:.8f}f" for j in range(cols))
        print(f"  {{{row}}},")
    print("};\n")


def generate_pid_ss(Kp, Ki, Kd, N, Ts, method="zoh", name_prefix=""):
    """
    Generate discrete state-space matrices for a filtered PID controller:

        C(s) = Kp + Ki/s + Kd * (N s)/(s + N)

    Continuous-time realization:
        x1_dot = e
        x2_dot = -N*x2 + N*e

        u = Ki*x1 - Kd*N*x2 + (Kp + Kd*N)*e

    Returns:
        Ad, Bd, C, D
    """

    # Continuous-time controller SS
    A = np.array([
        [0.0,  0.0],
        [0.0, -N]
    ], dtype=float)

    B = np.array([
        [1.0],
        [N]
    ], dtype=float)

    C = np.array([
        [Ki, -Kd * N]
    ], dtype=float)

    D = np.array([
        [Kp + Kd * N]
    ], dtype=float)

    # Discretize
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), Ts, method=method)

    # Pretty print summary
    print("============================================================")
    print("Filtered PID -> Discrete State Space")
    print("============================================================")
    print(f"Kp     = {Kp}")
    print(f"Ki     = {Ki}")
    print(f"Kd     = {Kd}")
    print(f"N      = {N}")
    print(f"Ts     = {Ts}")
    print(f"method = '{method}'")
    print()

    print("Continuous-time realization:")
    print("x1_dot = e")
    print(f"x2_dot = -{N}*x2 + {N}*e")
    print()
    print(f"u = {Ki}*x1 - ({Kd}*{N})*x2 + ({Kp} + {Kd}*{N})*e")
    print()

    print("Discrete matrices:")
    print("Ad =\n", Ad)
    print("Bd =\n", Bd)
    print("C  =\n", Cd)
    print("D  =\n", Dd)
    print()

    # C++ names
    Ad_name = f"{name_prefix}Adc" if name_prefix else "Adc"
    Bd_name = f"{name_prefix}Bdc" if name_prefix else "Bdc"
    C_name  = f"{name_prefix}Cdc"  if name_prefix else "Cdc"
    D_name  = f"{name_prefix}Ddc"  if name_prefix else "Ddc"

    print("// Ready-to-paste Arduino matrices")
    print_cpp_matrix(Ad_name, Ad)
    print_cpp_matrix(Bd_name, Bd)
    print_cpp_matrix(C_name, Cd)
    print_cpp_matrix(D_name, Dd)

    return Ad, Bd, Cd, Dd


def generate_second_order_plant_ss(wn, zeta, K, Ts, method="zoh"):
    """
    Generate discrete state-space matrices for the plant:

        y'' + 2*zeta*wn*y' + wn^2*y = K*u

    State definition:
        xp1 = y
        xp2 = y'

    Continuous-time form:
        x_dot = A x + B u
        y     = C x + D u
    """
    A = np.array([
        [0.0, 1.0],
        [-(wn**2), -2.0 * zeta * wn]
    ], dtype=float)

    B = np.array([
        [0.0],
        [K]
    ], dtype=float)

    C = np.array([
        [1.0, 0.0]
    ], dtype=float)

    D = np.array([
        [0.0]
    ], dtype=float)

    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), Ts, method=method)
    return Ad, Bd, Cd, Dd
