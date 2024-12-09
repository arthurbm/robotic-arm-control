import numpy as np
from math import cos, sin

def SE2_xy(x: float, y: float) -> np.ndarray:
    """
    Creates a 2D homogeneous transformation matrix for translation.
    
    Args:
        x (float): Translation along x-axis (in meters)
        y (float): Translation along y-axis (in meters)
    
    Returns:
        np.ndarray: 3x3 homogeneous transformation matrix
    """
    H = np.array([
        [1.0, 0.0, x],
        [0.0, 1.0, y],
        [0.0, 0.0, 1.0]
    ])
    return H

def SE2_theta(theta: float) -> np.ndarray:
    """
    Creates a 2D homogeneous transformation matrix for rotation.
    
    Args:
        theta (float): Rotation angle around origin (in radians)
    
    Returns:
        np.ndarray: 3x3 homogeneous transformation matrix
    """
    H = np.array([
        [cos(theta), -sin(theta), 0.0],
        [sin(theta),  cos(theta), 0.0],
        [0.0,         0.0,        1.0]
    ])
    return H

