import numpy as np
import math

def fk(theta1, theta2):
    """
    Forward Kinematics para um braço 2R planar com a1 = a2 = 1.
    Dado theta1 e theta2 (em radianos), retorna (x, y, theta).
    """
    a1 = 1.0
    a2 = 1.0
    
    # Posição final do efetuador
    x = a1 * math.cos(theta1) + a2 * math.cos(theta1 + theta2)
    y = a1 * math.sin(theta1) + a2 * math.sin(theta1 + theta2)
    # Orientação do efetuador final (soma dos ângulos)
    theta = theta1 + theta2
    
    return x, y, theta

def ik(x, y):
    """
    Inverse Kinematics para um braço 2R planar com a1 = a2 = 1.
    Dado (x, y), retorna (theta1, theta2) possíveis.
    """
    a1 = 1.0
    a2 = 1.0
    
    # Distância ao quadrado do ponto
    r2 = x**2 + y**2
    
    # Cálculo do ângulo theta2
    # cos(theta2) = (r2 - a1^2 - a2^2) / (2*a1*a2)
    # Como a1 = a2 = 1: cos(theta2) = (r2 - 2)/2
    cos_theta2 = (r2 - (a1**2 + a2**2)) / (2 * a1 * a2)
    
    # Verificar se cos_theta2 está no intervalo [-1,1]
    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        raise ValueError("Posição não alcançável, fora do espaço de trabalho.")
    
    # Há duas possíveis soluções: elbow-up ou elbow-down.
    # Vamos escolher a solução "elbow-up" (ou a que tenha sin positivo).
    sin_theta2 = math.sqrt(1 - cos_theta2**2)  # sempre positivo
    theta2 = math.atan2(sin_theta2, cos_theta2)
    
    # Agora calcular theta1
    # theta1 = atan2(y,x) - atan2(a2 sin(theta2), a1 + a2 cos(theta2))
    # Como a1 = a2 = 1: a1+a2*cos_theta2 = 1+cos_theta2
    # e a2*sin_theta2 = sin_theta2
    numerator = a2 * sin_theta2
    denominator = a1 + a2 * cos_theta2
    theta1 = math.atan2(y, x) - math.atan2(numerator, denominator)
    
    return theta1, theta2

# Exemplo de uso:
if __name__ == "__main__":
    # Exemplo direto: se theta1=30° (pi/6) e theta2=45° (pi/4)
    t1 = math.pi/6
    t2 = math.pi/4
    x, y, theta = fk(t1, t2)
    print("FK example:")
    print(f"Given theta1={t1:.4f} rad, theta2={t2:.4f} rad")
    print(f"Resulting end-effector: x={x:.4f}, y={y:.4f}, theta={theta:.4f}")

    # Agora usando IK no ponto obtido
    t1_calc, t2_calc = ik(x, y)
    print("\nIK example:")
    print(f"For x={x:.4f}, y={y:.4f}, recovered angles: theta1={t1_calc:.4f}, theta2={t2_calc:.4f}")
