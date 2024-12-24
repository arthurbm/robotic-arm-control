import numpy as np
import math

# Questão 1 - Funções de rotação e translação

def SE2_xy(x,y):
    H = np.eye(3)
    H[:2, 2] = [x, y]
    return H

def SE2_theta(theta):
    cos = np.cos(theta)
    sin = np.sin(theta)
    H = np.array([[cos,-sin, 0],
                   [sin, cos, 0],
                   [0,0,1]])  
    return H

# Função auxiliar
    # xi: posição relativa [vetor (x,y)]
    # theta: Angulo de rotação (rad)
    # p: ponto p no plano conhecido [vetor (x,y)]
    # in: Se deve calcular a inversa

    # return: matriz de coordenadas transformada

def transform(xi,theta, p, inv: bool = False):
    R = SE2_theta(theta)
    T = SE2_xy(xi[0], xi[1]) @ R

    if(inv):
        T = np.linalg.inv(T)

    p_hom = np.array([p[0], p[1], 1.0])
    return T @ p_hom


def fk(theta1, theta2):
    # tamanho dos dois links a = 1
    a = 1

    E = SE2_theta(theta1) @ SE2_xy(a,0) @ SE2_theta(theta2) @ SE2_xy(a, 0)
    return E[0][2], E[1][2], theta1+theta2

def ik(x,y):
    # tamanho dos dois links a = 1
    a = 1

    r2 = x**2 + y**2
    cos_theta2 = (r2 - (a**2 + a**2)) / (2 * a * a)
    # Checa se o valor é possível
    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        return "Posição inválida."
    
    sin_theta2 = math.sqrt(1 - cos_theta2**2)  
    theta2 = math.atan2(sin_theta2, cos_theta2)

    # Cálculo de theta1
    num = a * math.sin(theta2)
    denom = a + a * math.cos(theta2)
    theta1 = math.atan2(y, x) - math.atan2(num, denom)

    return theta1, theta2

# === TESTES ===

def runTests1():
    print("\n=== Início dos testes Q1 ===")
    #1
    xi = (1, 0.25)
    theta = 0
    p = (0.5, 0.5)
    result = transform(xi, theta, p)
    print(f"Resultado Teste 1: ({result[0]},{result[1]})")

    #2
    xi = (-1, -0.25)
    theta = 0
    p = (0.5, 0.5)
    result = transform(xi, theta, p)
    print(f"Resultado Teste 2: ({result[0]},{result[1]})")

    #3
    xi = (1, 0.25)
    theta = math.pi/4
    p = (0.5, 0.5)
    result = transform(xi, theta, p)
    print(f"Resultado Teste 3: ({result[0]},{result[1]})")

    #4
    xi = (1, 0.25)
    theta = math.pi/4
    p = (0.5, 0.5)
    result = transform(xi, theta, p, True)
    print(f"Resultado Teste 4: ({result[0]},{result[1]})")

    print("=== Fim dos testes Q1 ===\n")

def runTests2():
    print("\n=== Início dos testes Q2 ===")
    print("Testes de FK:")
    print(f"fk(0,pi/2): {fk(0,math.pi/2)}")
    print(f"fk(pi/2,pi/2): {fk(math.pi/2,math.pi/2)}")
    print(f"fk(pi/2,-pi/2): {fk(math.pi/2,-math.pi/2)}")
    print(f"fk(-pi,pi): {fk(-math.pi,math.pi)}")
    print("Testes de IK:")
    print(f"ik(1,1): {ik(1,1)}")
    print(f"ik(1,-1): {ik(1,-1)}")
    print(f"ik(-1,1): {ik(-1,1)}")
    print(f"ik(-1,-1): {ik(-1,-1)}")
    print(f"ik(2,1): {ik(2,1)}")
    print(f"ik(2,0): {ik(2,0)}")
    print(f"ik(0,2): {ik(0,2)}")
    print(f"ik(-2,0): {ik(-2,0)}")
    print("=== Fim dos testes Q2 ===\n")

runTests1()
runTests2()

