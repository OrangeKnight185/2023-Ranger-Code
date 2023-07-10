import numpy as np

NUM_OF_THRUSTERS = 8

T0D = -1
T1D = 1
T2D = 1
T3D = -1
T4D = 1
T5D = -1
T6D = -1
T7D = 1

THRUSTER_KEY_MATRIX = np.array([
    [ T0D,  T1D,  T2D,  T3D, -T4D, -T5D, -T6D, -T7D], # forward
    [ T0D, -T1D,  T2D, -T3D, -T4D,  T5D, -T6D,  T7D], # right
    [-T0D, -T1D,  T2D,  T3D, -T4D, -T5D,  T6D,  T7D], # up
    [ T0D, -T1D,  T2D, -T3D, -T4D,  T5D, -T6D,  T7D], # yaw right
    [-T0D, -T1D,  T2D,  T3D,  T4D,  T5D, -T6D, -T7D], # pitch up
    [ T0D, -T1D, -T2D,  T3D,  T4D, -T5D, -T6D,  T7D]]).T # roll right
# T:  0   1   2   3   4   5   6   7

THRUST_MAX_KG = 0.9 # 0.9

THRUSTER_DX = 90.32 / 1000.0
THRUSTER_DY = 90.46 / 1000.0
THRUSTER_DZ = 86.56 / 1000.0

THRUSTER_DISPLACEMENT_MATRIX = np.array([
    [THRUSTER_DX, THRUSTER_DY, THRUSTER_DZ], # T0
    [THRUSTER_DX, -THRUSTER_DY, THRUSTER_DZ], # T1
    [THRUSTER_DX, THRUSTER_DY, -THRUSTER_DZ], # T2
    [THRUSTER_DX, -THRUSTER_DY, -THRUSTER_DZ], # T3
    [-THRUSTER_DX, THRUSTER_DY, THRUSTER_DZ], # T4
    [-THRUSTER_DX, -THRUSTER_DY, THRUSTER_DZ], # T5
    [-THRUSTER_DX, THRUSTER_DY, -THRUSTER_DZ], # T6
    [-THRUSTER_DX, -THRUSTER_DY, -THRUSTER_DZ]]) # T7

THRUSTER_DIRECTION_COMPONENT = np.sqrt(3) / 3

THRUSTER_DIRECTION_MATRIX = np.array([
    [-THRUSTER_DIRECTION_COMPONENT, -THRUSTER_DIRECTION_COMPONENT, THRUSTER_DIRECTION_COMPONENT], # T0
    [-THRUSTER_DIRECTION_COMPONENT, THRUSTER_DIRECTION_COMPONENT, THRUSTER_DIRECTION_COMPONENT], # T1
    [-THRUSTER_DIRECTION_COMPONENT, -THRUSTER_DIRECTION_COMPONENT, -THRUSTER_DIRECTION_COMPONENT], # T2
    [-THRUSTER_DIRECTION_COMPONENT, THRUSTER_DIRECTION_COMPONENT, -THRUSTER_DIRECTION_COMPONENT], # T3
    [THRUSTER_DIRECTION_COMPONENT, -THRUSTER_DIRECTION_COMPONENT, THRUSTER_DIRECTION_COMPONENT], # T4
    [THRUSTER_DIRECTION_COMPONENT, THRUSTER_DIRECTION_COMPONENT, THRUSTER_DIRECTION_COMPONENT], # T5
    [THRUSTER_DIRECTION_COMPONENT, -THRUSTER_DIRECTION_COMPONENT, -THRUSTER_DIRECTION_COMPONENT], # T6
    [THRUSTER_DIRECTION_COMPONENT, THRUSTER_DIRECTION_COMPONENT, -THRUSTER_DIRECTION_COMPONENT]]) # T7

thruster_torque_array = []
for i in range(0, NUM_OF_THRUSTERS):
    thruster_torque_array.append(np.cross(THRUSTER_DISPLACEMENT_MATRIX[i], THRUSTER_DIRECTION_MATRIX[i]))
THRUSTER_TORQUE_MATRIX = np.array(thruster_torque_array).T

def torques_from_thrusters(thrusts: np.ndarray):
    return THRUSTER_TORQUE_MATRIX @ thrusts.T

def thrusts_from_thrusters(thrusts: np.ndarray):
    return THRUSTER_DIRECTION_MATRIX @ thrusts.T

def normalize(value, range_min, range_max, new_min, new_max):
    return (value - range_min) * (new_max - new_min) / (range_max - range_min) + new_min

def normalize_thrusters(raw_thrusters: np.ndarray):

    if raw_thrusters.max() > abs(raw_thrusters.min()):
        largest_element = raw_thrusters.max()
    else:
        largest_element = abs(raw_thrusters.min())

    if largest_element > 1.0:
        raw_thrusters = normalize(raw_thrusters, -largest_element, largest_element, -1.0, 1.0)
    
    return raw_thrusters

def thruster_percents(desired_movement: np.ndarray):
    thruster_values = THRUSTER_KEY_MATRIX @ desired_movement.T
    return normalize_thrusters(thruster_values).T

def thrust_to_pwm(thrust):
    if thrust > 0:
        pwm = -10.651 * (thrust ** 2) + 131.24 * thrust + 1546.9
    elif thrust < 0:
        pwm = 16.079 * (thrust ** 2) + 164.41 * thrust + 1453.5
    else:
        pwm = 1500
    return round(pwm)
v_thrust_to_pwm = np.vectorize(thrust_to_pwm)

def percent_to_pwm(thruster_percent):
    return thrust_to_pwm(thruster_percent * THRUST_MAX_KG)
v_percent_to_pwm = np.vectorize(percent_to_pwm)

def percent_to_thrust(thruster_percents):
    return thruster_percents * THRUST_MAX_KG
v_percent_to_thrust = np.vectorize(percent_to_thrust)

print(thruster_percents(np.array([0, 1, 0, 0, 0, 0])))
