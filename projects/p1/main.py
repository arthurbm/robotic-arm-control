import numpy as np
import math

def SE2_xy(x, y):
    """
    Cria uma matriz de transformação homogênea 2D para uma translação de (x, y).

    Parameters:
    x (float): Translação ao longo do eixo x
    y (float): Translação ao longo do eixo y

    Returns:
    np.ndarray: Matriz de transformação homogênea 3x3
    """
    H = np.array([
        [1, 0, x],
        [0, 1, y],
        [0, 0, 1]
    ], dtype=float)
    return H

def SE2_theta(theta):
    """
    Cria uma matriz de transformação homogênea 2D para uma rotação de theta em torno da origem.

    Parameters:
    theta (float): Ângulo de rotação em radianos

    Returns:
    np.ndarray: Matriz de transformação homogênea 3x3
    """
    c = math.cos(theta)
    s = math.sin(theta)

    H = np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ], dtype=float)
    return H


# Função auxiliar para aplicar uma matriz de transformação SE(2) em um ponto (x, y)
def transform_point(H, p):
    """
    Aplica uma transformação homogênea 2D a um ponto 2D.
    H: matriz 3x3 (np.ndarray)
    p: tupla ou lista (x, y)
    Retorna (x', y')
    """
    p_hom = np.array([p[0], p[1], 1.0])
    p_trans = H @ p_hom
    return p_trans[0], p_trans[1]

# Teste 1:
# Ponto conhecido em R2, queremos suas coordenadas em R1
# R2 está transladado de R1 por (1, 0.25) sem rotação.
P_R2 = (0.5, 0.5)
H_R1_from_R2 = SE2_xy(1, 0.25)  # R1 = R2 + (1,0.25)
P_R1_test1 = transform_point(H_R1_from_R2, P_R2)
print("1) P_R1 =", P_R1_test1)

# Teste 2:
# Ponto conhecido em R1, queremos suas coordenadas em R2 (apenas translação)
# R2 é R1 transladado por (1, 0.25), então inversamente R2 = R1 - (1,0.25)
P_R1 = (0.5, 0.5)
H_R2_from_R1 = SE2_xy(-1, -0.25) # Para ir de R1 para R2
P_R2_test2 = transform_point(H_R2_from_R1, P_R1)
print("2) P_R2 =", P_R2_test2)

# Teste 3:
# Agora R2 está a (1,0.25) com rotação de 45° em relação a R1.
# Queremos P em R1 a partir de P em R2.
P_R2 = (0.5, 0.5)
theta = math.radians(45)
H_R1_from_R2 = SE2_xy(1,0.25) @ SE2_theta(theta)  # R2 é rotacionado e então transladado
P_R1_test3 = transform_point(H_R1_from_R2, P_R2)
print("3) P_R1 =", P_R1_test3)

# Teste 4:
# Inverso do teste 3: conhecido P em R1, achar P em R2.
# A inversa de SE2_xy(1,0.25)*SE2_theta(45°) é SE2_theta(-45°)*SE2_xy(-1,-0.25)
P_R1 = (0.5,0.5)
H_R2_from_R1 = SE2_theta(-theta) @ SE2_xy(-1,-0.25) 
P_R2_test4 = transform_point(H_R2_from_R1, P_R1)
print("4) P_R2 =", P_R2_test4)