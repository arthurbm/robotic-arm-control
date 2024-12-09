import numpy as np
import math

def SE2_xy(x, y):
    """
    Creates a 2D homogeneous transformation matrix for a translation of (x, y).

    Parameters:
    x (float): Translation along x-axis
    y (float): Translation along y-axis

    Returns:
    np.ndarray: 3x3 homogeneous transformation matrix
    """
    H = np.array([
        [1, 0, x],
        [0, 1, y],
        [0, 0, 1]
    ], dtype=float)
    return H

def SE2_theta(theta):
    """
    Creates a 2D homogeneous transformation matrix for a rotation of theta around the origin.

    Parameters:
    theta (float): Rotation angle in radians

    Returns:
    np.ndarray: 3x3 homogeneous transformation matrix
    """
    c = math.cos(theta)
    s = math.sin(theta)

    H = np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ], dtype=float)
    return H


# Helper function to apply an SE(2) transformation matrix to a point (x, y)
def transform_point(H, p):
    """
    Applies a 2D homogeneous transformation to a 2D point.
    H: 3x3 matrix (np.ndarray)
    p: tuple or list (x, y)
    Returns (x', y')
    """
    p_hom = np.array([p[0], p[1], 1.0])
    p_trans = H @ p_hom
    return p_trans[0], p_trans[1]

# Test 1:
# Point known in R2, we want its coordinates in R1
# R2 is translated from R1 by (1, 0.25) without rotation.
P_R2 = (0.5, 0.5)
H_R1_from_R2 = SE2_xy(1, 0.25)  # R1 = R2 + (1,0.25)
P_R1_test1 = transform_point(H_R1_from_R2, P_R2)
print("1) P_R1 =", P_R1_test1)

# Test 2:
# Point known in R1, we want its coordinates in R2 (translation only)
# R2 is R1 translated by (1, 0.25), so inversely R2 = R1 - (1,0.25)
P_R1 = (0.5, 0.5)
H_R2_from_R1 = SE2_xy(-1, -0.25) # To go from R1 to R2
P_R2_test2 = transform_point(H_R2_from_R1, P_R1)
print("2) P_R2 =", P_R2_test2)

# Test 3:
# Now R2 is at (1,0.25) with a 45° rotation relative to R1.
# We want P in R1 from P in R2.
P_R2 = (0.5, 0.5)
theta = math.radians(45)
H_R1_from_R2 = SE2_xy(1,0.25) @ SE2_theta(theta)  # R2 is rotated and then translated
P_R1_test3 = transform_point(H_R1_from_R2, P_R2)
print("3) P_R1 =", P_R1_test3)

# Test 4:
# Inverse of test 3: known P in R1, find P in R2.
# The inverse of SE2_xy(1,0.25)*SE2_theta(45°) is SE2_theta(-45°)*SE2_xy(-1,-0.25)
P_R1 = (0.5,0.5)
H_R2_from_R1 = SE2_theta(-theta) @ SE2_xy(-1,-0.25) 
P_R2_test4 = transform_point(H_R2_from_R1, P_R1)
print("4) P_R2 =", P_R2_test4)