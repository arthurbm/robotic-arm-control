import numpy as np
import math
from ..q1.main import SE2_xy, SE2_theta

def fk(theta1, theta2):
    # length of both links a = 1
    a = 1

    E = SE2_theta(theta1) @ SE2_xy(a,0) @ SE2_theta(theta2) @ SE2_xy(a, 0)
    return E[0][2], E[1][2], theta1+theta2

def ik(x,y):
    # length of both links a = 1
    a = 1

    r2 = x**2 + y**2
    cos_theta2 = (r2 - (a**2 + a**2)) / (2 * a * a)
    # Check if the value is possible
    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        return "Invalid position."
    
    sin_theta2 = math.sqrt(1 - cos_theta2**2)  
    theta2 = math.atan2(sin_theta2, cos_theta2)

    # Calculation of theta1
    num = a * math.sin(theta2)
    denom = a + a * math.cos(theta2)
    theta1 = math.atan2(y, x) - math.atan2(num, denom)

    return theta1, theta2

def runTests():
    print("\n=== Start of Q2 Tests ===")
    print("FK Tests:")
    print(f"fk(0,pi/2): {fk(0,math.pi/2)}")
    print(f"fk(pi/2,pi/2): {fk(math.pi/2,math.pi/2)}")
    print(f"fk(pi/2,-pi/2): {fk(math.pi/2,-math.pi/2)}")
    print(f"fk(-pi,pi): {fk(-math.pi,math.pi)}")
    print("IK Tests:")
    print(f"ik(1,1): {ik(1,1)}")
    print(f"ik(1,-1): {ik(1,-1)}")
    print(f"ik(-1,1): {ik(-1,1)}")
    print(f"ik(-1,-1): {ik(-1,-1)}")
    print(f"ik(2,1): {ik(2,1)}")
    print(f"ik(2,0): {ik(2,0)}")
    print(f"ik(0,2): {ik(0,2)}")
    print(f"ik(-2,0): {ik(-2,0)}")
    print("=== End of Q2 Tests ===\n")

if __name__ == "__main__":
    runTests() 