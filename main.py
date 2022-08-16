import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import sys
import time

try:
    sys.path.append("/home/pi/SpiderPi_git/Functions/")
    import kinematics as kinematics
    import Board
except ImportError:
    pass


def cos(angle):
    return math.cos(math.radians(angle))


def acos(angle):
    return math.degrees(math.acos(angle))


def sin(angle):
    return math.sin(math.radians(angle))


def atan(ratio):
    return math.degrees(math.atan(ratio))


def generate_curve(
        curve_point=0.0,
        curve_type="circle",
        curve_period=1,
        curve_scale=1,
        curve_x_offset=0,
        curve_y_offset=0
):
    x_return = None
    y_return = None
    z_return = None
    if curve_period is not None:
        curve_point = curve_point % curve_period
    if curve_type == "circle":
        x_return = (math.cos(curve_point / curve_period * 2 * math.pi) * curve_scale) + curve_x_offset
        y_return = (math.sin(curve_point / curve_period * 2 * math.pi) * curve_scale) + curve_y_offset
    if curve_type == "sin":
        x_return = (math.sin(curve_point / curve_period * 2 * math.pi) * curve_scale) + curve_y_offset

    return x_return, y_return, z_return


class SpiderPi(object):
    hip_len = 44.6
    thigh_len = 75
    calf_len = 126.5

    leg_base_non_mid_x = 50
    leg_base_non_mid_y = 20
    leg_base_mid_y = 50

    hip_vertical_scale = 1.5
    knee_offset = -190
    knee_scale = 1.5

    # 0: front right
    # 1: mid right
    # 2: rear right
    # 3: rear left
    # 4: mid left
    # 5: front left
    leg_dict = {
        0: {
            # Front right
            "x": leg_base_non_mid_x,
            "y": leg_base_non_mid_y,
            "theta": 45,
            "hip_horiz_servo": 16,
            "hip_vertical_servo": 17,
            "knee_servo": 18,
            "invert_knee": True,
            "invert_hip": True,
        },
        1: {
            # Mid right
            "x": 0,
            "y": leg_base_mid_y,
            "theta": 90,
            "hip_horiz_servo": 13,
            "hip_vertical_servo": 14,
            "knee_servo": 15,
            "invert_knee": True,
            "invert_hip": True,
        },
        2: {
            # Rear right
            "x": -leg_base_non_mid_x,
            "y": leg_base_non_mid_y,
            "theta": 135,
            "hip_horiz_servo": 10,
            "hip_vertical_servo": 11,
            "knee_servo": 12,
            "invert_knee": True,
            "invert_hip": True,
        },
        3: {
            # Rear left
            "x": -leg_base_non_mid_x,
            "y": -leg_base_non_mid_y,
            "theta": -135,
            "hip_horiz_servo": 1,
            "hip_vertical_servo": 2,
            "knee_servo": 3,
            "invert_knee": False,
            "invert_hip": False,
        },
        4: {
            # Mid left
            "x": 0,
            "y": -leg_base_mid_y,
            "theta": -90,
            "hip_horiz_servo": 4,
            "hip_vertical_servo": 5,
            "knee_servo": 6,
            "invert_knee": False,
            "invert_hip": False,
        },
        5: {
            # Front left
            "x": leg_base_non_mid_x,
            "y": -leg_base_non_mid_y,
            "theta": -45,
            "hip_horiz_servo": 7,
            "hip_vertical_servo": 8,
            "knee_servo": 9,
            "invert_knee": False,
            "invert_hip": False,
        },
    }

    def __init__(self, servo_time=.05, walk_height=80, DEBUGGING=True):
        self.body_rotate = {
            "x": 0,
            "y": 0,
            "z": 0
        }
        self.body_shift = {
            "x": 0,
            "y": 0,
            "z": 0,
        }
        self.hip_angles = {x: 0.0 for x in range(6)}
        self.foot_locations = {x: {"x": 0, "y": 0, "z": 0} for x in range(6)}
        self.DEBUGGING = DEBUGGING
        distance_from_body = 170
        for curr_leg in range(6):
            curr_point = curr_leg / 6 + 1 / 12
            x_raw, y_raw, dummy = generate_curve(
                curve_point=curr_point,
                curve_scale=distance_from_body,
                curve_x_offset=0,
                curve_y_offset=0
            )
            z_raw = -walk_height
            self.foot_locations[curr_leg]["x"] = x_raw
            self.foot_locations[curr_leg]["y"] = y_raw
            self.foot_locations[curr_leg]["z"] = z_raw

        self.servo_time = int(1 * 1000)
        self.set_legs()
        self.servo_time = int(servo_time * 1000)
        time.sleep(1)

    def angle_to_servo(self, angle, offset_to_0=0, base_angle=0, invert_servo=False, angle_scale=1.0):
        angle = base_angle - angle
        angle = self.normalize_angle(angle)
        angle *= angle_scale
        resulting_angle = 500 + (angle / 180) * 500 * (-1 if invert_servo else 1)
        resulting_angle -= offset_to_0 * (-1 if invert_servo else 1)
        if resulting_angle < 0:
            resulting_angle = 0
        if resulting_angle > 1000:
            resulting_angle = 1000
        return int(resulting_angle)

    @staticmethod
    def normalize_angle(angle):
        while angle > 180:
            angle -= 180
        while angle < -180:
            angle += 180
        return angle

    def unload_servos(self):
        for curr_leg in range(6):
            Board.unloadBusServo(self.leg_dict[curr_leg]["hip_horiz_servo"])
            time.sleep(.03)
            Board.unloadBusServo(self.leg_dict[curr_leg]["hip_vertical_servo"])
            time.sleep(.03)
            Board.unloadBusServo(self.leg_dict[curr_leg]["knee_servo"])
            time.sleep(.03)

    def set_servo_time(self, servo_time):
        self.servo_time = int(servo_time * 1000)

    def set_body_rotate(self, axis, angle):
        if axis == "z":
            for curr_leg in self.generate_leg_set():
                x_raw = self.foot_locations[curr_leg]["x"]
                y_raw = self.foot_locations[curr_leg]["y"]
                z_raw = self.foot_locations[curr_leg]["z"]
                rot_mat = R.from_rotvec([
                    0,
                    0,
                    math.radians(self.body_rotate["z"]) - math.radians(angle)
                ]).as_matrix()

                x_raw -= self.get_body_shift("x")
                y_raw -= self.get_body_shift("y")
                z_raw -= self.get_body_shift("z")

                x_raw, y_raw, z_raw = np.matmul(
                    rot_mat,
                    [x_raw, y_raw, z_raw]
                )
                self.foot_locations[curr_leg]["x"] = x_raw
                self.foot_locations[curr_leg]["y"] = y_raw
                self.foot_locations[curr_leg]["z"] = z_raw
        self.body_rotate[axis] = angle

    def get_body_rotate(self, axis):
        return self.body_rotate[axis]

    def shift_body_rotate(self, axis, angle):
        self.set_body_rotate(axis, self.get_body_rotate(axis) + angle)

    def set_body_shift(self, axis, distance):
        self.body_shift[axis] = distance

    def get_body_shift(self, axis):
        return self.body_shift[axis]

    def shift_body_shift(self, axis, distance):
        self.set_body_shift(axis, self.get_body_shift(axis) + distance)

    @staticmethod
    def generate_leg_set(right=False, left=False, odd=False, even=False, front=False, mid=False, rear=False):
        if not right and not left and not odd and not even and not front and not mid and not rear:
            return list(range(6))
        right_set = {0, 1, 2}
        left_set = {3, 4, 5}
        odd_set = {1, 3, 5}
        even_set = {0, 2, 4}
        front_set = {0, 5}
        mid_set = {1, 4}
        rear_set = {2, 3}
        and_list = []
        if right:
            and_list.append(right_set)
        if left:
            and_list.append(left_set)
        if odd:
            and_list.append(odd_set)
        if even:
            and_list.append(even_set)
        if front:
            and_list.append(front_set)
        if mid:
            and_list.append(mid_set)
        if rear:
            and_list.append(rear_set)

        ret_set = and_list[0]
        for curr_set in and_list[1:]:
            ret_set &= curr_set

        return sorted(ret_set)

    def set_leg_shift(self, leg_list, axis, distance):
        for curr_leg in leg_list:
            self.foot_locations[curr_leg][axis] += distance

    def set_legs(self):
        for curr_leg in range(6):
            # x_raw = leg_base_non_mid_x
            # y_raw = -leg_base_non_mid_y - thigh_len - hip_len
            #     if curr_leg < 3:
            #         y_raw = 130
            #     else:
            #         y_raw = -130
            # print(f"Leg {curr_leg}, X {x_raw} Y {y_raw}")
            x_raw = self.foot_locations[curr_leg]["x"]
            y_raw = self.foot_locations[curr_leg]["y"]
            z_raw = self.foot_locations[curr_leg]["z"]
            rot_mat = R.from_rotvec([
                math.radians(self.body_rotate["x"]),
                math.radians(self.body_rotate["y"]),
                math.radians(self.body_rotate["z"])
            ]).as_matrix()

            x_raw -= self.get_body_shift("x")
            y_raw -= self.get_body_shift("y")
            z_raw -= self.get_body_shift("z")

            x_raw, y_raw, z_raw = np.matmul(
                rot_mat,
                [x_raw, y_raw, z_raw]
            )

            x_final = x_raw
            y_final = y_raw
            z_final = z_raw

            base_x = self.leg_dict[curr_leg]["x"]
            base_y = self.leg_dict[curr_leg]["y"]
            base_theta = self.leg_dict[curr_leg]["theta"]
            x_delta = x_final - base_x
            y_delta = y_final - base_y
            z_delta = z_final - 0
            angle = self.hip_angles[curr_leg]
            if x_delta == 0:
                if y_delta > 0:
                    angle = 90
                if y_delta < 0:
                    angle = -90
            elif y_delta == 0:
                if x_delta > 0:
                    angle = 0
                if x_delta < 0:
                    angle = 180
            elif x_delta > 0:
                angle = atan(y_delta / x_delta)
            elif x_delta < 0:
                if y_delta > 0:
                    angle = atan(y_delta / x_delta) + 180
                if y_delta < 0:
                    angle = atan(y_delta / x_delta) - 180

            hip_horizontal_servo_angle = self.angle_to_servo(angle, 0, base_theta)

            horizontal_leg = ((x_delta ** 2 + y_delta ** 2) ** .5) - self.hip_len

            if horizontal_leg < 0:
                horizontal_leg = 10

            leg_distance = (horizontal_leg ** 2 + z_delta ** 2) ** .5

            if leg_distance > self.thigh_len + self.calf_len:
                print("Leg target too far")
                leg_distance = self.thigh_len + self.calf_len - 10
                horizontal_leg = (leg_distance ** 2 - z_delta ** 2) ** .5

            # Distance from hip to foot less than calf-thigh
            if leg_distance < self.calf_len - self.thigh_len:
                print("Leg target too close for calf minus thigh")
                leg_distance = self.calf_len - self.thigh_len + 10
                # target is too close so may need to extend horizontal_leg_distance
                if leg_distance > horizontal_leg + abs(z_delta):
                    print("Also need to re-calculate horizontal leg distance")
                    horizontal_leg = leg_distance - abs(z_delta) + 10

            # Angle from "thigh" to "path between hip and toe"
            hip_total_angle = acos(
                (leg_distance ** 2 + self.thigh_len ** 2 - self.calf_len ** 2) / (2 * leg_distance * self.thigh_len))
            # Angle from "thigh" to "path between hip and toe"
            hip_offset_angle = acos(
                (leg_distance ** 2 + horizontal_leg ** 2 - z_delta ** 2) / (2 * leg_distance * horizontal_leg))
            # Angle between "thigh" and "horizontal"
            if z_delta <= 0:
                hip_angle = hip_total_angle - hip_offset_angle
            else:
                hip_angle = hip_total_angle + hip_offset_angle
            #     hip_vertical_servo_angle = angle_to_servo(hip_angle, invert_servo=leg_dict[curr_leg]["invert_hip"])

            hip_vertical_servo_angle = self.angle_to_servo(
                hip_angle,
                invert_servo=self.leg_dict[curr_leg]["invert_hip"],
                angle_scale=self.hip_vertical_scale
            )

            knee_angle = acos(
                (self.thigh_len ** 2 + self.calf_len ** 2 - leg_distance ** 2) / (2 * self.thigh_len * self.calf_len))
            knee_angle = 180 - knee_angle
            #     knee_servo_angle = angle_to_servo(knee_angle, invert_servo=leg_dict[curr_leg]["invert_knee"])

            knee_servo_angle = self.angle_to_servo(
                knee_angle,
                invert_servo=self.leg_dict[curr_leg]["invert_knee"],
                offset_to_0=self.knee_offset,
                angle_scale=self.knee_scale
            )

            self.hip_angles[curr_leg] = angle
            if self.DEBUGGING:
                print(f"Leg: {curr_leg}")
                print(f"X: {x_delta}")
                print(f"Y: {y_delta}")
                print(f"Z: {z_delta}")
                print(f"Horizontal distance: {horizontal_leg}")
                print(f"Leg distance: {leg_distance}")
                print(f"Hip horizontal angle: {angle}")
                print(f"Hip horizontal servo angle: {hip_horizontal_servo_angle}")
            if do_servos:
                # time.sleep(.03)
                Board.setBusServoPulse(self.leg_dict[curr_leg]["hip_horiz_servo"], int(hip_horizontal_servo_angle),
                                       self.servo_time)
            if self.DEBUGGING:
                print(f"Hip vertical angle: {hip_angle}")
                print(f"Hip vertical servo angle: {hip_vertical_servo_angle}")
            if do_servos:
                # time.sleep(.03)
                Board.setBusServoPulse(self.leg_dict[curr_leg]["hip_vertical_servo"], int(hip_vertical_servo_angle),
                                       self.servo_time)
            if self.DEBUGGING:
                print(f"Knee angle: {knee_angle}")
                print(f"Knee servo angle: {knee_servo_angle}")
            if do_servos:
                # time.sleep(.03)
                Board.setBusServoPulse(self.leg_dict[curr_leg]["knee_servo"], int(knee_servo_angle), self.servo_time)

    def simple_step(self, step_time=None, z_step=30, x_distance_per_step=80, y_distance_per_step=0, num_steps=1):
        if step_time is None:
            sleep_time = self.servo_time
        else:
            sleep_time = step_time
            self.set_servo_time(step_time)
        self.set_leg_shift(self.generate_leg_set(even=True), axis="z", distance=z_step)
        self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
        self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
        self.set_legs()
        time.sleep(sleep_time)
        self.set_leg_shift(self.generate_leg_set(even=True), axis="x", distance=x_distance_per_step / 2)
        self.set_leg_shift(self.generate_leg_set(even=True), axis="y", distance=y_distance_per_step / 2)
        self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
        self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
        self.set_legs()
        time.sleep(sleep_time)
        self.set_leg_shift(self.generate_leg_set(even=True), axis="z", distance=-z_step)
        self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
        self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
        self.set_legs()
        time.sleep(sleep_time)
        for step in range(num_steps):
            self.set_leg_shift(self.generate_leg_set(odd=True), axis="z", distance=z_step)
            self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
            self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
            self.set_legs()
            time.sleep(sleep_time)
            self.set_leg_shift(self.generate_leg_set(odd=True), axis="x", distance=x_distance_per_step)
            self.set_leg_shift(self.generate_leg_set(odd=True), axis="y", distance=y_distance_per_step)
            self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
            self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
            self.set_legs()
            time.sleep(sleep_time)
            self.set_leg_shift(self.generate_leg_set(odd=True), axis="z", distance=-z_step)
            self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
            self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
            self.set_legs()
            time.sleep(sleep_time)
            self.set_leg_shift(self.generate_leg_set(even=True), axis="z", distance=z_step)
            self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
            self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
            self.set_legs()
            time.sleep(sleep_time)
            self.set_leg_shift(self.generate_leg_set(even=True), axis="x", distance=x_distance_per_step)
            self.set_leg_shift(self.generate_leg_set(even=True), axis="y", distance=y_distance_per_step)
            self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
            self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
            self.set_legs()
            time.sleep(sleep_time)
            self.set_leg_shift(self.generate_leg_set(even=True), axis="z", distance=-z_step)
            self.shift_body_shift(axis="x", distance=x_distance_per_step / 6)
            self.shift_body_shift(axis="y", distance=y_distance_per_step / 6)
            self.set_legs()
            time.sleep(sleep_time)
        self.set_leg_shift(self.generate_leg_set(odd=True), axis="z", distance=z_step)
        self.set_legs()
        time.sleep(sleep_time)
        self.set_leg_shift(self.generate_leg_set(odd=True), axis="x", distance=x_distance_per_step / 2)
        self.set_leg_shift(self.generate_leg_set(odd=True), axis="y", distance=y_distance_per_step / 2)
        self.set_legs()
        time.sleep(sleep_time)
        self.set_leg_shift(self.generate_leg_set(odd=True), axis="z", distance=-z_step)
        self.set_legs()
        time.sleep(sleep_time)


do_servos = False
# print(SpiderPi.generate_leg_set(mid=True))
spider = SpiderPi(servo_time=.05)

spider.set_legs()
# for i in range(100):
#     step = i/100
#     angle, dummy, dummy1 = generate_curve(curve_point=step, curve_type="sin", curve_scale=20)
#     spider.set_body_rotate("x", angle)
#     spider.set_legs()
#     time.sleep(1)

# curr_point = curr_leg / 6 + 1 / 12
# x_raw, y_raw = generate_curve(
#     curve_point=curr_point,
#     curve_scale=distance_from_body,
#     curve_x_offset=0,
#     curve_y_offset=0
# )
# z_raw = -60
# if leg_mod is not None:
#     if curr_leg % 2 != leg_mod:
#         continue
