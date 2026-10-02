'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 1A of Niti Vahan (NV) Theme.
*
*  Filename:         ackermann_steering.py
*  Created:          2026
*  Last Modified:
*  Author:           e-Yantra Team
*
*  You are ONLY allowed to write your code inside the block marked
*  "ADD YOUR IMPLEMENTATION HERE". Do not change anything outside it - the
*  evaluation script relies on the rest of this file staying as it is.
*
*****************************************************************************************
'''

# Team ID:          <eYRC#4102>
# Author List:      <Santhosh M>
# Filename:         ackermann_steering.py
# Functions:        ackermann_wheel_angles
# Global variables: < List any global variables you add, "None" if you add none >


####################### IMPORT MODULES #######################
import math
import numpy as np
##############################################################


#################### VEHICLE CONSTANTS #######################
WHEELBASE = 0.120           # L: distance between front and rear axle centrelines
TRACK_WIDTH = 0.110         # W: distance between left and right wheel centre
WHEEL_OFFSET = 0.0275       # O: distance between kingpin axis and wheel centre.
##############################################################


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################


def ackermann_wheel_angles(delta):
    '''
    Purpose:
    ---
    Convert a single virtual steering angle into the two real front-wheel
    angles, per the Ackermann geometry.

    Input Arguments:
    ---
    `delta` :   [ float ]
        Steering angle of the virtual centred front wheel, in radians.

    Returns:
    ---
    `left_angle`  : [ float ]
    `right_angle` : [ float ]
        The two real front-wheel steering angles, in radians, using the
        same sign convention as delta.

    REMEMBER:
    ---
    WHEEL_OFFSET changes the effective half-track width inside each wheel's triangle.
    '''

    # Handle straight driving (delta approx 0)
    if abs(delta) < 1e-6:
        return 0.0, 0.0

    # Effective track width accounting for kingpin offset
    W_eff = TRACK_WIDTH - 2 * WHEEL_OFFSET
    L = WHEELBASE

    # Calculate cotangent of magnitude of delta
    cot_delta = 1.0 / math.tan(abs(delta))

    # Inner and Outer cotangents
    cot_inner = cot_delta - (W_eff / L)
    cot_outer = cot_delta + (W_eff / L)

    # Convert back to angles (radians)
    inner_angle = math.atan(1.0 / cot_inner)
    outer_angle = math.atan(1.0 / cot_outer)

    # Apply sign conventions:
    # Positive delta -> Turning Left (Left is inner, Right is outer)
    # Negative delta -> Turning Right (Left is outer, Right is inner)
    if delta > 0:
        left_angle = inner_angle
        right_angle = outer_angle
    else:
        left_angle = -outer_angle
        right_angle = -inner_angle

    return left_angle, right_angle

##############################################################
################ END OF YOUR IMPLEMENTATION ##################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################

if __name__ == "__main__":

    test_angles = np.arange(-0.35, 0.35, 0.05)

    for d in test_angles:
        left, right = ackermann_wheel_angles(d)
        print(f"delta={d}  ->  left={left}, right={right}")
