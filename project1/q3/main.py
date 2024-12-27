import numpy as np
import math
import roboticstoolbox as rtb
from ..q2.main import ik

# Global parameters
TS = 11        # Number of discrete samples (steps)
V_MAX = 1.0
A_MAX = 1.0

def get_q_vector(t, t_total, t_acc, t_cte, q_init, delta):
    """
    Returns the position array for one joint, given:
      t: time array
      t_total: total motion time
      t_acc: time spent accelerating
      t_cte: time at constant velocity
      q_init: initial angle
      delta: total angle change (final - initial)
    """
    q = np.zeros_like(t)
    t_deacc = t_acc + t_cte  # time we start decelerating
    d_acc = 0.5 * A_MAX * (t_acc**2)
    v_now = A_MAX * t_acc

    for i, ti in enumerate(t):
        if ti <= t_acc:  
            # Acceleration phase
            q[i] = q_init + 0.5 * A_MAX * (ti**2) * np.sign(delta)
        elif ti <= t_deacc:
            # Constant velocity phase
            q[i] = q_init + (d_acc + v_now * (ti - t_acc)) * np.sign(delta)
        else:
            # Deceleration phase
            t_left = t_total - ti
            # from final => minus deceleration from v=0
            q[i] = (q_init + delta) - 0.5 * A_MAX * (t_left**2) * np.sign(delta)
    return q

def traj_joint(theta1_init, theta2_init, theta1_final, theta2_final):
    """
    Joint-space trajectory that ensures both joints finish at the same time.
    Follows a trapezoid approach with possible constant-velocity phase.
    """
    # Distances to move for each joint
    d1 = theta1_final - theta1_init
    d2 = theta2_final - theta2_init

    # Time to accelerate from 0 to V_MAX
    t_acc_max = V_MAX / A_MAX
    d_acc_max = 0.5 * A_MAX * (t_acc_max**2)  # distance covered in acc or dec

    # For each joint, check if it can reach V_MAX or not
    def calc_times(d):
        dist_abs = abs(d)
        if dist_abs < 2 * d_acc_max:
            # No constant velocity phase
            t_acc = math.sqrt(dist_abs / A_MAX)
            t_cte = 0
        else:
            # We reach V_MAX
            t_acc = t_acc_max
            dist_cte = dist_abs - 2*d_acc_max
            t_cte = dist_cte / V_MAX
        return t_acc, t_cte

    t_acc1, t_cte1 = calc_times(d1)
    t_acc2, t_cte2 = calc_times(d2)

    t_total1 = 2*t_acc1 + t_cte1
    t_total2 = 2*t_acc2 + t_cte2
    t_total = max(t_total1, t_total2)

    # Extend the shorter motion to match total time
    if t_total1 < t_total:
        t_cte1 += (t_total - t_total1)
    if t_total2 < t_total:
        t_cte2 += (t_total - t_total2)

    # Time array: we have TS discrete steps in [0, t_total]
    t = np.linspace(0, t_total, TS)

    q1 = get_q_vector(t, t_total, t_acc1, t_cte1, theta1_init, d1)
    q2 = get_q_vector(t, t_total, t_acc2, t_cte2, theta2_init, d2)

    return np.column_stack((q1, q2))

def traj_eucl(x_init, y_init, x_final, y_final):
    """
    Euclidian-space trajectory. Moves from (x_init,y_init) to (x_final,y_final)
    along a straight line. For simplicity, we match the same TS steps and unify time 
    for the motion. It uses the same approach of finishing in t_total.
    """
    dx = x_final - x_init
    dy = y_final - y_init
    dist = math.hypot(dx, dy)

    if dist < 1e-9:
        # No movement
        t1, t2 = ik(x_init, y_init)
        return np.array([[t1, t2]])
    
    # We'll compute the total motion time in the same style as the joint approach
    t_acc_max = V_MAX / A_MAX
    d_acc_max = 0.5 * A_MAX * (t_acc_max**2)

    # Check if we reach V_MAX in the linear path or not
    if dist < 2*d_acc_max:
        t_acc = math.sqrt(dist / A_MAX)
        t_cte = 0
    else:
        t_acc = t_acc_max
        dist_cte = dist - 2*d_acc_max
        t_cte = dist_cte / V_MAX

    t_total = 2*t_acc + t_cte
    t = np.linspace(0, t_total, TS)

    # In each phase, param s(t) is how far along the path we are from x_init,y_init
    # We'll build s(t) similarly to get_q_vector, but simpler since it's 1D
    s = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti <= t_acc:
            s[i] = 0.5 * A_MAX * (ti**2)
        elif ti <= t_acc + t_cte:
            s[i] = d_acc_max + V_MAX * (ti - t_acc)
        else:
            t_left = t_total - ti
            s[i] = dist - 0.5 * A_MAX * (t_left**2)

    # Now convert each point (px, py) to joint angles
    path_q = np.zeros((TS, 2))
    for i, si in enumerate(s):
        frac = si / dist
        px = x_init + dx * frac
        py = y_init + dy * frac
        t1, t2 = ik(px, py)
        path_q[i] = [t1, t2]
    return path_q

if __name__ == "__main__":
    robot = rtb.models.DH.Planar2()
    print("Choose a trajectory to visualize:")
    print("[1] Joint-space trajectory (ex.: from (0,60°) to (0,30°))")
    print("[2] Euclidian-space trajectory (ex.: from (0,0) to (1,1))")
    choice = input("Enter your choice (1/2): ")

    if choice == '1':
        # Example: from t1=0°, t2=60° to t1=0°, t2=30°
        t1_init = math.radians(0)
        t2_init = math.radians(60)
        t1_final = math.radians(0)
        t2_final = math.radians(30)
        qj = traj_joint(t1_init, t2_init, t1_final, t2_final)
        print("Joint-space shape:", qj.shape)
        robot.plot(qj, backend='pyplot', movie='joint_space.gif')

    elif choice == '2':
        # Euclidian from (0,0) to (1,1)
        qe = traj_eucl(0, 0, 1, 1)
        print("Euclidian-space shape:", qe.shape)
        robot.plot(qe, backend='pyplot', movie='eucl_space.gif')

    else:
        print("Invalid choice. Exiting.")
