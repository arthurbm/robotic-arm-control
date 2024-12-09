import math

def fk(theta1, theta2):
    """
    Forward Kinematics of a planar 2R robot with a1 = a2 = 1.
    Given theta1 and theta2 (in rad), returns (x, y, theta).

    Equations:
    x = a1*cos(theta1) + a2*cos(theta1+theta2)
    y = a1*sin(theta1) + a2*sin(theta1+theta2)
    theta = theta1 + theta2
    """
    a1 = 1.0
    a2 = 1.0
    
    x = a1 * math.cos(theta1) + a2 * math.cos(theta1 + theta2)
    y = a1 * math.sin(theta1) + a2 * math.sin(theta1 + theta2)
    theta = theta1 + theta2
    
    return x, y, theta

def ik(x, y):
    """
    Inverse Kinematics of a planar 2R robot with a1 = a2 = 1.
    Given (x, y), returns (theta1, theta2).

    Formulas:
    cos(theta2) = (x² + y² - a1² - a2²)/(2*a1*a2)
    Here a1=a2=1, so:
    cos(theta2) = (x² + y² - 2)/2

    theta2 = atan2(+sqrt(1 - cos²(theta2)), cos(theta2)) (choosing "elbow-up" solution)

    theta1 = atan2(y, x) - atan2(a2*sin(theta2), a1 + a2*cos(theta2))
    With a1=a2=1:
    theta1 = atan2(y, x) - atan2(sin(theta2), 1+cos(theta2))
    """
    a1 = 1.0
    a2 = 1.0

    r2 = x**2 + y**2
    cos_theta2 = (r2 - (a1**2 + a2**2)) / (2 * a1 * a2)
    
    # Check if the position is reachable
    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        raise ValueError("Position not reachable, outside the robot's workspace.")
    
    # Two possible solutions: we choose positive sin (elbow-up)
    sin_theta2 = math.sqrt(1 - cos_theta2**2)  
    theta2 = math.atan2(sin_theta2, cos_theta2)

    # Cálculo de theta1
    numerator = a2 * math.sin(theta2)
    denominator = a1 + a2 * math.cos(theta2)
    theta1 = math.atan2(y, x) - math.atan2(numerator, denominator)

    return theta1, theta2

# Usage example
if __name__ == "__main__":
    # FK test
    t1 = math.pi/6   # 30°
    t2 = math.pi/4   # 45°
    x, y, theta = fk(t1, t2)
    print("FK Example:")
    print(f"Given theta1={t1:.4f}, theta2={t2:.4f}")
    print(f"End-effector -> x={x:.4f}, y={y:.4f}, theta={theta:.4f}")

    # IK test
    t1_calc, t2_calc = ik(x, y)
    print("\nIK Example:")
    print(f"For x={x:.4f}, y={y:.4f}, recovered angles: theta1={t1_calc:.4f}, theta2={t2_calc:.4f}")
