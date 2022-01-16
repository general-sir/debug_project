import numpy as np
import math
from scipy.spatial.transform import Rotation as R

def cos(angle):
    return math.cos(math.radians(angle))


def sin(angle):
    return math.sin(math.radians(angle))


def atan(ratio):
    return math.degrees(math.atan(ratio))


def generate_rotation_matrix(curr_angle):
    return [
        [cos(curr_angle), sin(curr_angle), 0],
        [-sin(curr_angle), cos(curr_angle), 0],
        [0, 0, 1]
    ]


leg_dict = {
    0: {
        "x": 15,
        "y": 20,
        "theta": 45
    }
}

# Translate point

angle = 69

x_raw = 20
y_raw = 20
z_raw = -10

body_x_rotate = 0
body_y_rotate = 0
body_z_rotate = 0

hip_len = 44.6
thigh_len = 75
calf_len = 126.5

curr_leg = 0

rot_mat = R.from_rotvec([
    math.radians(body_x_rotate),
    math.radians(body_y_rotate),
    math.radians(body_z_rotate)
]).as_matrix()

x_raw, y_raw, z_raw = np.matmul(
    rot_mat,
    [x_raw, y_raw, z_raw]
)

x_final = x_raw
y_final = y_raw
z_final = z_raw

base_x = leg_dict[curr_leg]["x"]
base_y = leg_dict[curr_leg]["y"]
base_theta = leg_dict[curr_leg]["theta"]
x_delta = x_final - base_x
y_delta = y_final - base_y
z_delta = z_final - 0
if x_delta == 0:
    if y_delta > 0:
        angle = 90
    if y_delta < 0:
        angle = 270
elif y_delta == 0:
    if x_delta > 0:
        angle = 0
    if x_delta < 0:
        angle = 180
elif x_delta > 0:
    angle = atan(y_delta / x_delta)
elif x_delta < 0:
    angle = atan(y_delta / x_delta) + 180

horizontal_leg = ((x_delta**2 + y_delta**2) ** .5) - hip_len

leg_distance = (horizontal_leg**2 + z_delta**2) ** .5

if leg_distance > thigh_len + calf_len:
    print("Leg target too far")
    leg_distance = thigh_len + calf_len - 1

if leg_distance + thigh_len < calf_len:
    leg_distance =

hip_ratio = (thigh_len**2 + leg_distance**2 - calf_len**2) / (2*thigh_len*leg_distance)
hip_angle = math.acos(hip_ratio)

print(f"X: {x_delta}")
print(f"Y: {y_delta}")
print(f"Angle: {angle}")