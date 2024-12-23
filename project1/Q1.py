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
    # theta: Angulo de rotação 
    # p: ponto p no plano conhecido [vetor (x,y)]
    # in: Se deve calcular a inversa

    # return: matriz de coordenadas transformada

def transform(xi,theta, p, inv: bool = False):
    t = math.radians(theta)
    R = SE2_theta(t)
    T = SE2_xy(xi[0], xi[1]) @ R

    if(inv):
        T = np.linalg.inv(T)

    p_hom = np.array([p[0], p[1], 1.0])
    return T @ p_hom

# === TESTES ===

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
theta = 45
p = (0.5, 0.5)
result = transform(xi, theta, p)
print(f"Resultado Teste 3: ({result[0]},{result[1]})")

#4
xi = (1, 0.25)
theta = 45
p = (0.5, 0.5)
result = transform(xi, theta, p, True)
print(f"Resultado Teste 4: ({result[0]},{result[1]})")
