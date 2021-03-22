import time
from abc import ABCMeta, abstractmethod
from math import atan, atan2, cos, degrees, hypot, isclose, isnan, radians, inf, sin
from typing import Any, List, NewType, Optional, Tuple

import vrepapi.sim as vrepapi

from L2.src.main import VREPClient
from L4.src.main import PID

VREPHandle = NewType('VREPHandle', Any)


def constrain(val, min_, max_):
    return max(min_, min(val, max_))


class ModelController:
    client_id: int

    _basename: str
    _sep: str

    def init(self):
        self._init()

    @classmethod
    def _construct_name(cls, *args) -> str:
        return cls._sep.join([cls._basename, *args])


class ManipulatorController(ModelController):
    _basename = 'manipulator'
    _sep = '_'
    __link_basename = 'link'
    __joint_basename = 'joint'
    __sensor_basename = 'sensor'

    __n_links = 4

    __base_handle: VREPHandle
    __links_handles: List[VREPHandle]
    __joint_handles: List[VREPHandle]
    __tip_handle: VREPHandle

    __prox_sensor_handle: VREPHandle
    __vis_sensor_handle: VREPHandle

    __D_EPS: float = 1e-2
    __ANGLE_EPS: float = 0.1
    __G_THRESH: float = 0.9
    __RB_THRESH: float = 0.3

    __search_start: float = None
    __init_pos: Tuple[float] = None

    range: float = 1.5

    @classmethod
    def get_base_name(cls) -> str:
        return cls.get_link_name(0)

    @classmethod
    def get_link_name(cls, n: int) -> str:
        return cls._construct_name(f'{cls.__link_basename}{n}')

    @classmethod
    def get_joint_name(cls, n: int) -> str:
        """
        :param n: index of the link that is controlled by the joint
        :return: name of the joint
        """
        links = [f'{cls.__link_basename}{_}' for _ in (n, n + 1)]
        return cls._construct_name(cls.__joint_basename, *links)

    @classmethod
    def get_tip_name(cls):
        return cls._construct_name('tip')

    @classmethod
    def get_controller_name(cls):
        return cls._construct_name('control')

    @classmethod
    def get_sensor_name(cls, type: str) -> str:
        return cls._construct_name(cls.__sensor_basename, type)

    @property
    def handle(self):
        return self.__base_handle

    def _init(self):
        self.__init_handles()
        self.__init_datastreams()

    def __init_handles(self):
        self.__base_handle, *self.__links_handles = [
                vrepapi.simxGetObjectHandle(self.client_id, self.get_link_name(_), vrepapi.simx_opmode_blocking)[1]
                for _ in range(self.__n_links)]
        self.__joint_handles = [
                vrepapi.simxGetObjectHandle(self.client_id, self.get_joint_name(_), vrepapi.simx_opmode_blocking)[1] for
                _ in range(self.__n_links - 1)]
        _, self.__tip_handle = vrepapi.simxGetObjectHandle(
                self.client_id, self.get_tip_name(), vrepapi.simx_opmode_blocking
                )
        _, self.__prox_sensor_handle = vrepapi.simxGetObjectHandle(
                self.client_id, self.get_sensor_name('proximity'), vrepapi.simx_opmode_blocking
                )
        _, self.__vis_sensor_handle = vrepapi.simxGetObjectHandle(
                self.client_id, self.get_sensor_name('visual'), vrepapi.simx_opmode_blocking
                )
        _, self.__controller_handle = vrepapi.simxGetObjectHandle(
                self.client_id, self.get_controller_name(), vrepapi.simx_opmode_blocking)

        _, self.__object = vrepapi.simxGetObjectHandle(
                self.client_id, 'Object', vrepapi.simx_opmode_blocking)

        _, self.__gripper = vrepapi.simxGetObjectHandle(
                self.client_id, 'gripper_joint_right', vrepapi.simx_opmode_blocking)

    @property
    def object_(self):
        return self.__object

    def clench(self):
        print('Clenching...')
        vrepapi.simxSetJointTargetVelocity(self.client_id, self.__gripper, 0.2, vrepapi.simx_opmode_blocking)
        vrepapi.simxSetJointForce(self.client_id, self.__gripper, 20, vrepapi.simx_opmode_blocking)
        time.sleep(3)
        print('Done.')

    def unclench(self):
        print('Unclenching...')
        vrepapi.simxSetJointTargetVelocity(self.client_id, self.__gripper, -0.2, vrepapi.simx_opmode_blocking)
        vrepapi.simxSetJointForce(self.client_id, self.__gripper, 20, vrepapi.simx_opmode_blocking)
        time.sleep(3)
        print('Done.')

    def __init_datastreams(self):
        vrepapi.simxReadProximitySensor(self.client_id, self.__prox_sensor_handle, vrepapi.simx_opmode_streaming)
        vrepapi.simxReadVisionSensor(self.client_id, self.__vis_sensor_handle, vrepapi.simx_opmode_streaming)

    def set_target_pose(self, position: Tuple[float, float, float], parent: VREPHandle = -1):
        vrepapi.simxSetObjectPosition(
                self.client_id, self.__controller_handle, parent, position, vrepapi.simx_opmode_blocking)

    def get_target_pose(self):
        while True:
            res, pos = vrepapi.simxGetObjectPosition(
                    self.client_id, self.__controller_handle, self.handle, vrepapi.simx_opmode_blocking)
            if res == 0:
                return pos

    def __move_next_point(self):
        t = time.time()
        if self.__search_start is None:
            self.__search_start = t
        R = .25
        wxy = .7
        wz = 3
        z = .9 - .1 * sin(wz * (t - self.__search_start))
        x = R * cos(wxy * (t - self.__search_start))
        y = R * sin(wxy * (t - self.__search_start))
        self.set_target_pose((x, y, z), self.handle)

    def search_object(self) -> VREPHandle:
        found = False
        while not found:
            self.__move_next_point()
            resv, detectedv, packet = vrepapi.simxReadVisionSensor(
                    self.client_id, self.__vis_sensor_handle, vrepapi.simx_opmode_buffer)
            resp, detectedp, (_, _, d), handle, normal = vrepapi.simxReadProximitySensor(
                    self.client_id, self.__prox_sensor_handle, vrepapi.simx_opmode_buffer)

            if not (VREPClient.response_good(resp) and VREPClient.response_good(resv)):
                continue
            if not (packet and detectedp):
                continue

            r, g, b = packet[0][-4:-1]
            found = g > (r + b)

        return handle

    def get_position(self):
        res, pos = vrepapi.simxGetObjectPosition(self.client_id, self.__base_handle, -1, vrepapi.simx_opmode_blocking)
        return pos

    def get_effector_pos(self):
        res, pos = vrepapi.simxGetObjectPosition(self.client_id, self.__tip_handle, self.handle,
                                                 vrepapi.simx_opmode_blocking)
        return pos


class VehicleController(ModelController):
    _basename = 'vehicle'
    _sep = '_'
    __joint_basename = 'joint'
    __wheel_basename = 'wheel'
    __steer_basename = 'steer'
    __drive_basename = 'drive'
    __sensor_basename = 'sensor'

    __side_configurations = [
            [fr, side] for fr in ('front', 'rear') for side in ('left', 'right')
            ]

    __max_dir = 45
    __max_vel = 360
    __dir_pid = PID(-__max_dir, __max_dir, 1000, 10, 0, 0)
    __vel_pid = PID(-__max_vel, __max_vel, 10, 10, 0, 0)

    @property
    def handle(self):
        return self.__base_handle

    @classmethod
    def __get_wheel_name(cls, fr, side):
        return cls._construct_name(cls.__wheel_basename, fr, side)

    @classmethod
    def __get_joint_name(cls, type: str, fr: str, side: str):
        return cls._construct_name(cls.__joint_basename, type, fr, side)

    @classmethod
    def __get_drive_joint_name(cls, fr, side):
        return cls.__get_joint_name(cls.__drive_basename, fr, side)

    @classmethod
    def __get_steer_joint_name(cls, fr, side):
        return cls.__get_joint_name(cls.__steer_basename, fr, side)

    @classmethod
    def __get_base_name(cls) -> str:
        return cls._construct_name('base')

    @classmethod
    def __get_sensor_name(cls, side: str) -> str:
        return cls._construct_name(cls.__sensor_basename, side)

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
        self.trunk: VREPHandle = vrepapi.simxGetObjectHandle(
                self.client_id, self._construct_name('trunk'), vrepapi.simx_opmode_blocking
                )[1]

    def __init_datastreams(self):
        for wheel in self.__wheel_handles:
            vrepapi.simxGetObjectVelocity(self.client_id, wheel, vrepapi.simx_opmode_streaming)
        vrepapi.simxReadProximitySensor(self.client_id, self.__left_sensor, vrepapi.simx_opmode_streaming)
        vrepapi.simxReadProximitySensor(self.client_id, self.__right_sensor, vrepapi.simx_opmode_streaming)

    def _init(self):
        self.__init_handles()
        self.__init_datastreams()

    def __estimate_dir(self, ld, rd):
        max_ = 2
        min_ = 20e-3

        ld = constrain(ld, min_, max_)
        rd = constrain(rd, min_, max_)

        s = (ld + rd) / 2
        d = (ld - rd) / s * self.__max_dir

        print(d, ld, rd)

        return constrain(d, -self.__max_dir, self.__max_dir)

    def __estimate_vel(self, ld, rd):
        max_ = 2
        min_ = 20e-3

        ld = constrain(ld, min_, max_)
        rd = constrain(rd, min_, max_)

        vel = (1 - abs(ld - rd) * .5 / (max_ - min_)) * (ld + rd) * .5 / (max_ - min_) * self.__max_vel

        return constrain(vel, -self.__max_vel, self.__max_vel)

    def __get_wheel_velocity(self, wheel_handle: VREPHandle):
        _, lin, ang = vrepapi.simxGetObjectVelocity(self.client_id, wheel_handle, vrepapi.simx_opmode_buffer)
        return sum([_ * _ for _ in ang[:2]]) ** .5 if VREPClient.response_good(_) else 0

    def move(self, direction: float):
        ld = self.__get_distance(self.__left_sensor)
        rd = self.__get_distance(self.__right_sensor)

        est_vel = self.__estimate_vel(ld, rd)
        est_dir = self.__estimate_dir(ld, rd)

        self.__set_vehicle_dir(est_dir)
        self.__set_vehicle_vel(est_vel, est_dir)

        return direction

    @property
    def direction(self):
        res, pose = vrepapi.simxGetObjectOrientation(self.client_id, self.__base_handle, -1,
                                                     vrepapi.simx_opmode_blocking)
        return degrees(pose[2]) if VREPClient.response_good(res) else 0

    def __set_vehicle_dir(self, direction: float):
        for joint in self.__steer_joints:
            self.__set_joint_pos(joint, radians(direction))

    def __set_joint_pos(self, joint_handle: VREPHandle, pos: float):
        # print(f'Setting joint pos: {pos}')
        vrepapi.simxSetJointTargetPosition(self.client_id, joint_handle, pos, vrepapi.simx_opmode_oneshot)

    def __set_vehicle_vel(self, vel: float, direction: float = 0):
        left = 1 - direction/self.__max_dir
        right = 1 + direction/self.__max_dir
        self.__set_joint_vel(self.__drive_joints[0], radians(vel) * left)
        self.__set_joint_vel(self.__drive_joints[1], radians(vel) * right)
        self.__set_joint_vel(self.__drive_joints[2], radians(vel) * left)
        self.__set_joint_vel(self.__drive_joints[3], radians(vel) * right)

    def __set_joint_vel(self, joint_handle: VREPHandle, vel: float):
        vrepapi.simxSetJointTargetVelocity(self.client_id, joint_handle, vel, vrepapi.simx_opmode_oneshot)

    def __get_distance(self, sensor_handle: VREPHandle) -> Optional[float]:
        __max_dist = 2
        # print(f'Reading from proximity sensor...')

        res, ready, (_, _, d), *_ = \
            vrepapi.simxReadProximitySensor(
                    self.client_id,
                    sensor_handle,
                    vrepapi.simx_opmode_buffer
                    )

        # print(f'Readings:\n'
        #       f'  response: {res}; ready: {ready}; distance: {d}\n')

        return d if VREPClient.response_good(res) and ready else __max_dist

    def stop(self):
        while not isclose(self.__get_wheel_velocity(self.__wheel_handles[-1]), 0, abs_tol=1e-3):
            self.__set_vehicle_vel(0)

    def get_velocity(self):
        _, lin, ang = vrepapi.simxGetObjectVelocity(self.client_id, self.__base_handle, vrepapi.simx_opmode_buffer)
        return sum([_ * _ for _ in lin[:2]]) ** .5 if VREPClient.response_good(_) else 0

    def get_position(self):
        res, pos = vrepapi.simxGetObjectPosition(self.client_id, self.__base_handle, -1, vrepapi.simx_opmode_blocking)
        return pos


class L5Controller(VREPClient):
    manipulator: ManipulatorController
    vehicle: VehicleController

    __known_handles: List[VREPHandle] = list()

    def __init(self):
        self.manipulator.client_id = self.client_id
        self.manipulator.init()
        self.vehicle.client_id = self.client_id
        self.vehicle.init()

    def __move_to_target(self):
        direction = self.__target_dir()
        self.vehicle.move(direction)

    def __get_target_pos(self, handle):
        res, target_pos = vrepapi.simxGetObjectPosition(self.client_id, handle, vehicle.handle,
                                                        vrepapi.simx_opmode_oneshot_wait)
        return target_pos if VREPClient.response_good(res) else 0

    def __target_dir(self) -> float:
        direction = atan2(self.__target_pos[1], self.__target_pos[0])
        direction = degrees(direction)

        print('Direction to object: \n'
              f'\ttarget position: {self.__target_pos}\n'
              f'\tvehicle position: {self.__vehicle_pos}\n'
              f'\tdirection: {direction}\n')

        return direction

    def __in_range(self):
        try:
            return hypot(self.__target_pos[0], self.__target_pos[1]) < self.manipulator.range
        except AttributeError:
            return False

    def __load_object(self, handle: VREPHandle):
        pos = self.__get_target_pos(handle)
        pos[2] += .5
        self.manipulator.set_target_pose(pos, vehicle.handle)
        prev_pos = (0, 0, 0)
        while True:
            mp = self.manipulator.get_effector_pos()
            dp = sum((m - t) ** 2 for m, t in zip(mp, prev_pos)) ** .5
            prev_pos = mp
            if isclose(dp, 0, abs_tol=3e-2):
                break
            time.sleep(.5)
        self.manipulator.set_target_pose((0, 0, .25), handle)
        time.sleep(2)
        self.manipulator.clench()
        time.sleep(3)
        self.manipulator.set_target_pose((0, 0, 1), handle)
        time.sleep(3)
        self.manipulator.set_target_pose((0, 1, 1), self.manipulator.handle)
        time.sleep(3)
        trunk_pos = self.__get_target_pos(self.vehicle.trunk)
        self.manipulator.set_target_pose((*trunk_pos[:2], 1), self.vehicle.handle)
        time.sleep(3)
        self.manipulator.set_target_pose(trunk_pos, self.vehicle.handle)
        time.sleep(3)
        self.manipulator.unclench()

    def _run(self):
        self.__init()
        handle: VREPHandle = self.manipulator.search_object()
        # handle: VREPHandle = self.manipulator.object_
        while not self.__in_range():
            self.__target_pos = self.__get_target_pos(handle)
            self.__vehicle_pos = self.vehicle.get_position()
            self.__move_to_target()
        self.vehicle.stop()
        self.__load_object(handle)


if __name__ == '__main__':
    SERVER_HOST = '127.0.0.1'
    SERVER_PORT = 19999

    manip = ManipulatorController()
    vehicle = VehicleController()

    controller = L5Controller(SERVER_HOST, SERVER_PORT)

    controller.manipulator = manip
    controller.vehicle = vehicle

    controller.run()
