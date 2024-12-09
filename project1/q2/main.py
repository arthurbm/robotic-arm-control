import numpy as np
import math

def fk(theta1, theta2):
    """
    Cinemática direta (Forward Kinematics) de um robô 2R planar com a1 = a2 = 1.
    Dado theta1 e theta2 (em rad), retorna (x, y, theta).

    Equações:
    x = a1*cos(theta1) + a2*cos(theta1+theta2)
    y = a1*sin(theta1) + a2*sin(theta1+theta2)
    theta = theta1 + theta2
    """
    a1 = 1.0
    a2 = 1.0
    
    x = a1 * np.cos(theta1) + a2 * np.cos(theta1 + theta2)
    y = a1 * np.sin(theta1) + a2 * np.sin(theta1 + theta2)
    theta = theta1 + theta2
    
    return float(x), float(y), float(theta)

def ik(x, y):
    """
    Cinemática inversa (Inverse Kinematics) de um robô 2R planar com a1 = a2 = 1.
    Dado (x, y), retorna (theta1, theta2).

    Fórmulas:
    cos(theta2) = (x² + y² - a1² - a2²)/(2*a1*a2)
    Aqui a1=a2=1, então:
    cos(theta2) = (x² + y² - 2)/2

    theta2 = atan2(+sqrt(1 - cos²(theta2)), cos(theta2)) (escolhendo solução "cotovelo para cima")

    theta1 = atan2(y, x) - atan2(a2*sin(theta2), a1 + a2*cos(theta2))
    Com a1=a2=1:
    theta1 = atan2(y, x) - atan2(sin(theta2), 1+cos(theta2))
    """
    a1 = 1.0
    a2 = 1.0

    r2 = x**2 + y**2
    cos_theta2 = (r2 - (a1**2 + a2**2)) / (2 * a1 * a2)
    
    # Verifica se a posição é alcançável
    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        raise ValueError("Posição não alcançável, fora do espaço de trabalho do robô.")
    
    # Duas possíveis soluções: escolhemos sin positivo (elbow-up)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)  
    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # Cálculo de theta1
    numerator = a2 * sin_theta2
    denominator = a1 + a2 * cos_theta2
    theta1 = np.arctan2(y, x) - np.arctan2(numerator, denominator)

    return float(theta1), float(theta2)

# Exemplo de uso
if __name__ == "__main__":
    # Teste da fk
    t1 = math.pi/6   # 30°
    t2 = math.pi/4   # 45°
    x, y, theta = fk(t1, t2)
    print("Exemplo FK:")
    print(f"Given theta1={t1:.4f}, theta2={t2:.4f}")
    print(f"End-effector -> x={x:.4f}, y={y:.4f}, theta={theta:.4f}")

    # Teste da ik
    t1_calc, t2_calc = ik(x, y)
    print("\nExemplo IK:")
    print(f"For x={x:.4f}, y={y:.4f}, recovered angles: theta1={t1_calc:.4f}, theta2={t2_calc:.4f}")
