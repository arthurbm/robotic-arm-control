import numpy as np
import math
# from ..q1.main import SE2_theta, SE2_xy
# from ..q2.main import ik,fk
import roboticstoolbox as rtb

TS = 11 
V_MAX = 1.0
A_MAX = 1.0
T_ACC_MAX = V_MAX/A_MAX
D_ACC = (A_MAX*(T_ACC_MAX**2))/2

def get_q_vector(t, t_total, t_acc, t_cte, theta_inicial, delta):
    q = np.zeros_like(t)

    t_deacc = t_acc + t_cte
    d_acc = (A_MAX*(t_acc**2))/2
    v_atual = A_MAX*t_acc

    for i, time in enumerate(t):
        if(time <= t_acc): # Acelerando
            q[i] = theta_inicial + 0.5* (A_MAX*(time**2)) * np.sign(delta)
        elif(time > t_acc and time < t_deacc): # Velocidade constante
            q[i] = theta_inicial + (d_acc + v_atual * (time-t_acc)) * np.sign(delta)
        elif(time >= t_deacc): # Desacelerando
            t_dacc = t_total - time
            q[i] = (delta +theta_inicial) - ( + 0.5* (A_MAX*(t_dacc**2))) * np.sign(delta)
    
    return q


def traj_joint(theta1_init, theta2_init, theta1_final, theta2_final):
    # Distâncias a percorrer
    d1 = theta1_final - theta1_init
    d2 = theta2_final - theta2_init
    
    # Tempo necessário para atingir V_MAX com aceleração constante
    #     
    # Verificar se é possível atingir V_MAX ou apenas acelerar/desacelerar
    if d1 < 2 * D_ACC:
        # q1 não atinge V_MAX: resolve para nova aceleração limitada
        t_acc1 = np.sqrt(abs(d1) / A_MAX)
        t_cte1 = 0
    else:
        # q1 atinge V_MAX
        t_acc1 = T_ACC_MAX
        d_cte1 = d1 - 2 * D_ACC
        t_cte1 = d_cte1 / V_MAX
    
    if d2 < 2 * D_ACC:
        # q2 não atinge V_MAX
        t_acc2 = np.sqrt(abs(d2) / A_MAX)
        t_cte2 = 0
    else:
        # q2 atinge V_MAX
        t_acc2 = T_ACC_MAX
        d_cte2 = d2 - 2 * D_ACC
        t_cte2 = d_cte2 / V_MAX
    
    # Tempo total de viagem para cada junta
    t_total1 = 2 * t_acc1 + t_cte1
    t_total2 = 2 * t_acc2 + t_cte2
    
    # Ajustar para que ambas cheguem ao mesmo tempo
    t_total = max(t_total1, t_total2)
    if t_total1 < t_total:
        t_cte1 += t_total - t_total1
    elif t_total2 < t_total:
        t_cte2 += t_total - t_total2  

    t = np.arange(0, t_total, (t_total)/TS)    

    # print(f"d1 {d1}")
    # print(f"d2 {d2}")
    # print(f"t_acc {t_acc1},       {t_acc2}")
    # print(f"t_cte {t_cte1},       {t_cte2}")
    # print(f"t_total {t_total}")
    # print(t)

    q1 = get_q_vector(t, t_total, t_acc1, t_cte1, theta1_init, d1)
    q2 = get_q_vector(t, t_total, t_acc2, t_cte2, theta2_init, d2)

    return np.column_stack((q1,q2))

def traj_eucl(x_init, y_init, x_final, y_final):

    return

# Create the RR robot object
robot = rtb.models.DH.Planar2 ()

t1_inicial = np.radians(0)
t2_inicial = np.radians(60)
t1_final = np.radians(00)
t2_final = np.radians(30)

qt = traj_joint(t1_inicial,t2_inicial,t1_final,t2_final)
# print(qt)

# Visualize the trajectory
robot . plot ( qt , backend = 'pyplot' , movie = 'RR.gif')