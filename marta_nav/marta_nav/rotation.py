#!/usr/bin/env python3

from scipy.spatial.transform import Rotation as R
import numpy as np

def main():
    r = R.from_rotvec(np.pi/2 * np.array([0, 0, -1]))
    print(r.as_matrix())
    r2 = R.from_rotvec(np.pi/2 * np.array([-1, 0, 0]))
    print(r2.as_matrix())
    r3 = R.from_rotvec(np.pi/2 * np.array([0, 0, -1]))
    r = R.from_matrix(np.dot(r.as_matrix(),r2.as_matrix()))
    #r = R.from_matrix(np.dot(r.as_matrix(),r3.as_matrix()))
    
    print(r.as_quat())
    

if __name__ == "__main__":
    main()