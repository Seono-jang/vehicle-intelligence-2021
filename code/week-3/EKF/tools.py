import numpy as np
from math import sqrt
from math import atan2

def Jacobian(x):
    px, py, vx, vy = x
    if px == 0 and py == 0:
        print("Error: both px and py are zero while trying to")
        print("       calculate the Jacobian.")
        return np.zeros((3, 4))
    # Prepare calculation
    c1 = px * px + py * py
    c2 = sqrt(c1)
    c3 = c1 * c2
    # Fill in the matrix
    Hj = np.array([
        [px / c2, py / c2, 0.0, 0.0],
        [-py / c1, px / c1, 0.0, 0.0],
        [py * (vx * py - vy * px) / c3,
         px * (vy * px - vx * py) / c3,
         px / c2,
         py / c2]
    ])
    return Hj

def output_matrix(x):
    px, py, vx, vy = x
    if px == 0 and py == 0:
        print("Error: both px and py are zero while trying to")
        print("       calculate the output matrix.")
        return np.zeros(3)
    H_x = np.array([
        sqrt(px*px + py*py),
        atan2(py,px),
        (px * vx + py * vy) / sqrt(px * px + py * py)
    ])
     
    if H_x[1] < 0 :
        H_x[1] = H_x[1] + 2*np.pi
         
    return H_x