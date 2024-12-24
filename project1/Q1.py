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


# === TESTES ===

def runTests1():
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

def runTests2():
    print(fk(0,math.pi/2))
    print(fk(math.pi/2,math.pi/2))
    print(fk(math.pi/2,-math.pi/2))
    print(fk(-math.pi,math.pi))

runTests2()
runTests1()

