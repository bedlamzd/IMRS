import time
from math import isnan, radians, inf
from typing import Any, List, NewType, Optional

import vrepapi.sim as vrepapi

from L2.src.main import VREPClient

VREPHandle = NewType('VREPHandle', Any)


class PID:
    __min: float
    __max: float

    __kp: float
    __ki: float
    __kd: float

    __last_err: float
    __int_err: float
    __out: float

    def __init__(self, min_: float, max_: float, dt: float, kp: float = 1, ki: float = 0, kd: float = 0):
        self.__min = min_
        self.__max = max_

        self.__kp = kp
        self.__ki = ki
        self.__kd = kd

        self.__last_time = 0
        self.sample_time = dt

        self.__last_err = 0
        self.__int_err = 0
        self.__out = 0

    @property
    def __int(self) -> float:
        return self.__int_err

    @__int.setter
    def __int(self, val: float):
        self.__int_err = constrain(val, self.__min, self.__max)

    @property
    def __output(self):
        return self.__out

    @__output.setter
    def __output(self, val: float):
        self.__out = constrain(val, self.__min, self.__max)

    def __call__(self, goal: float, current: float) -> float:
        cur_time = time.time() * 10e3

        if cur_time - self.__last_time > self.sample_time:
            current = 0 if isnan(current) else current
            err = 0 if isnan(_ := (goal - current)) else _

            self.__int += self.sample_time * err
            d_err = err - self.__last_err

            self.__output = self.__kp * err + self.__ki * self.__int + self.__kd * d_err

            self.__last_err = err
            self.__last_time = cur_time

        return self.__output


def constrain(val, min_, max_):
    return max(min_, min(val, max_))


class L4Controller(VREPClient):
    __basename = 'vehicle'
    __sep = '_'
    __joint_basename = 'joint'
    __wheel_basename = 'wheel'
    __steer_basename = 'steer'
    __drive_basename = 'drive'
    __sensor_basename = 'sensor'

    __side_configurations = [
            [fr, side] for fr in ('front', 'rear') for side in ('left', 'right')
            ]

    __max_dir = 45
    __max_vel = 2000
    __dir_pid = PID(-__max_dir, __max_dir, 100, 250, .01, -5)
    __vel_pid = PID(100, __max_vel, 100, 30, .001, .01)

    @classmethod
    def __construct_name(cls, *args) -> str:
        return cls.__sep.join([cls.__basename, *args])

    @classmethod
    def __get_wheel_name(cls, fr, side):
        return cls.__construct_name(cls.__wheel_basename, fr, side)

    @classmethod
    def __get_joint_name(cls, type: str, fr: str, side: str):
        return cls.__construct_name(cls.__joint_basename, type, fr, side)

    @classmethod
    def __get_drive_joint_name(cls, fr, side):
        return cls.__get_joint_name(cls.__drive_basename, fr, side)

    @classmethod
    def __get_steer_joint_name(cls, fr, side):
        return cls.__get_joint_name(cls.__steer_basename, fr, side)

    @classmethod
    def __get_base_name(cls) -> str:
        return cls.__construct_name('base')

    @classmethod
    def __get_sensor_name(cls, side: str) -> str:
        return cls.__construct_name(cls.__sensor_basename, side)

    def __init_handles(self):
        self.__base_handle: VREPHandle = vrepapi.simxGetObjectHandle(
                self.client_id, self.__get_base_name(), vrepapi.simx_opmode_blocking)[1]

        self.__wheel_handles: List[VREPHandle] = [
                vrepapi.simxGetObjectHandle(self.client_id, self.__get_wheel_name(*_), vrepapi.simx_opmode_blocking)[1]
                for _ in self.__side_configurations
                ]

        self.__drive_joints: List[VREPHandle] = [
                vrepapi.simxGetObjectHandle(self.client_id, self.__get_drive_joint_name(*args),
                                            vrepapi.simx_opmode_blocking)[1]
                for args in self.__side_configurations
                ]
        self.__steer_joints: List[VREPHandle] = [
                vrepapi.simxGetObjectHandle(self.client_id, self.__get_steer_joint_name(fr, side),
                                            vrepapi.simx_opmode_blocking)[1]
                for fr, side in self.__side_configurations if fr == 'front'
                ]
        self.__left_sensor: VREPHandle = vrepapi.simxGetObjectHandle(
                self.client_id, self.__get_sensor_name('left'), vrepapi.simx_opmode_blocking
                )[1]
        self.__right_sensor: VREPHandle = vrepapi.simxGetObjectHandle(
                self.client_id, self.__get_sensor_name('right'), vrepapi.simx_opmode_blocking
                )[1]

    def __init_datastreams(self):
        for wheel in self.__wheel_handles:
            vrepapi.simxGetObjectVelocity(self.client_id, wheel, vrepapi.simx_opmode_streaming)
        vrepapi.simxReadProximitySensor(self.client_id, self.__left_sensor, vrepapi.simx_opmode_streaming)
        vrepapi.simxReadProximitySensor(self.client_id, self.__right_sensor, vrepapi.simx_opmode_streaming)

    def __init(self):
        self.__init_handles()
        self.__init_datastreams()

    def __get_distance(self, sensor_handle: VREPHandle) -> Optional[float]:
        __max_dist = 2
        print(f'Reading from proximity sensor...')

        res, ready, (_, _, d), *_ = \
            vrepapi.simxReadProximitySensor(
                    self.client_id,
                    sensor_handle,
                    vrepapi.simx_opmode_buffer
                    )

        print(f'Readings:\n'
              f'  response: {res}; ready: {ready}; distance: {d}\n')

        return d if self.response_good(res) and ready else __max_dist

    def get_joint_pos(self, joint_handle: VREPHandle):
        _, curr_coord = vrepapi.simxGetJointPosition(self.client_id, joint_handle, vrepapi.simx_opmode_blocking)
        # print(f'Getting joint pos... pos: {curr_coord}')
        return curr_coord if self.response_good(_) else None

    def move_joint(self, joint_handle: VREPHandle, delta: float):
        curr_pos = self.get_joint_pos(joint_handle)
        if curr_pos is None:
            return
        self.__set_joint_pos(joint_handle, curr_pos + delta)

    def __get_wheel_velocity(self, wheel_handle: VREPHandle):
        _, lin, ang = vrepapi.simxGetObjectVelocity(self.client_id, wheel_handle, vrepapi.simx_opmode_buffer)
        return sum([_ * _ for _ in ang[:2]]) ** .5 if self.response_good(_) else 0

    def __move(self):
        d = self.direction
        v = self.velocity
        self.__set_vehicle_dir(d)
        self.__set_vehicle_vel(v, d)

    def __set_vehicle_dir(self, direction: float):
        for joint in self.__steer_joints:
            self.__set_joint_pos(joint, radians(direction))

    def __set_joint_pos(self, joint_handle: VREPHandle, pos: float):
        print(f'Setting joint pos: {pos}')
        vrepapi.simxSetJointTargetPosition(self.client_id, joint_handle, pos, vrepapi.simx_opmode_oneshot)

    @property
    def direction(self) -> float:
        ld = self.__get_distance(self.__left_sensor)
        rd = self.__get_distance(self.__right_sensor)

        max_ = 2
        min_ = 20e-3

        ld = constrain(ld, min_, max_)
        rd = constrain(rd, min_, max_)

        ld = (max_ - ld) / (ld * max_)
        rd = (max_ - rd) / (rd * max_)

        ld = ld * min_ * max_ / (max_ - min_)
        rd = rd * min_ * max_ / (max_ - min_)

        goal = 0
        u = self.__dir_pid(goal, ld - rd)

        print('Calculating direction:\n'
              f'\tdirection: {u}\n'
              f'\tinput: {rd - ld}')

        return u

    def __set_vehicle_vel(self, vel: float, direction: float = 0):
        left = 1 - direction / self.__max_dir
        right = 1 + direction / self.__max_dir
        self.__set_joint_vel(self.__drive_joints[0], radians(vel) * left)
        self.__set_joint_vel(self.__drive_joints[1], radians(vel) * right)
        self.__set_joint_vel(self.__drive_joints[2], radians(vel) * left)
        self.__set_joint_vel(self.__drive_joints[3], radians(vel) * right)

    def __set_joint_vel(self, joint_handle: VREPHandle, vel: float):
        vrepapi.simxSetJointTargetVelocity(self.client_id, joint_handle, vel, vrepapi.simx_opmode_oneshot)

    @property
    def velocity(self) -> float:
        ld = self.__get_distance(self.__left_sensor)
        rd = self.__get_distance(self.__right_sensor)

        max_ = 2
        min_ = 20e-3

        ld = constrain(ld, min_, max_)
        rd = constrain(rd, min_, max_)

        ld_v = (ld - min_) / (max_ - min_)
        rd_v = (rd - min_) / (max_ - min_)

        ld_d = (max_ - ld) / (ld * max_)
        rd_d = (max_ - rd) / (rd * max_)

        ld_d = ld_d * min_ * max_ / (max_ - min_)
        rd_d = rd_d * min_ * max_ / (max_ - min_)

        goal = min(self.__max_vel * ((ld_v + rd_v) / 2 - abs((ld_d - rd_d) / 2)), self.__max_vel)
        u = self.__vel_pid(goal, self.__get_wheel_velocity(self.__wheel_handles[-1]))

        print('Calculating velocity:\n'
              f'\tvelocity: {u}')

        return u

    def _run(self):
        self.__init()
        while True:
            self.__move()


if __name__ == '__main__':
    SERVER_HOST = '127.0.0.1'
    SERVER_PORT = 19999

    controller = L4Controller(SERVER_HOST, SERVER_PORT)

    controller.run()
