import numpy as np
import math

# Question 1 - Rotation and Translation Functions

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

# Helper Function
    # xi: relative position [vector (x,y)]
    # theta: rotation angle (rad)
    # p: known point p in the plane [vector (x,y)]
    # inv: whether to calculate the inverse

    # return: transformed coordinate matrix

def transform(xi,theta, p, inv: bool = False):
    R = SE2_theta(theta)
    T = SE2_xy(xi[0], xi[1]) @ R

    if(inv):
        T = np.linalg.inv(T)

    p_hom = np.array([p[0], p[1], 1.0])
    return T @ p_hom

def runTests():
    print("\n=== Start of Q1 Tests ===")
    #1
    xi = (1, 0.25)
    theta = 0
    p = (0.5, 0.5)
    result = transform(xi, theta, p)
    print(f"Test Result 1: ({result[0]},{result[1]})")

    #2
    xi = (-1, -0.25)
    theta = 0
    p = (0.5, 0.5)
    result = transform(xi, theta, p)
    print(f"Test Result 2: ({result[0]},{result[1]})")

    #3
    xi = (1, 0.25)
    theta = math.pi/4
    p = (0.5, 0.5)
    result = transform(xi, theta, p)
    print(f"Test Result 3: ({result[0]},{result[1]})")

    #4
    xi = (1, 0.25)
    theta = math.pi/4
    p = (0.5, 0.5)
    result = transform(xi, theta, p, True)
    print(f"Test Result 4: ({result[0]},{result[1]})")

    print("=== End of Q1 Tests ===\n")

if __name__ == "__main__":
    runTests() 