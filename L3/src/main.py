from typing import Any, List, NewType, Optional
from math import pi, isclose, degrees, radians

import vrepapi.sim as vrepapi

from L2.src.main import VREPClient

VREPHandle = NewType('VREPHandle', Any)


class L3Controller(VREPClient):
    __basename = 'manipulator'
    __sep = '_'
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

    __joint_ranges: List[float] = [
            [None, None],
            [-pi/3, pi/2],
            [0, pi/3]
            ]

    __joint_dp: List[float] = [
            .08,
            .08,
            .08
            ]

    @classmethod
    def __construct_name(cls, *args) -> str:
        return cls.__sep.join([cls.__basename, *args])

    @classmethod
    def get_base_name(cls) -> str:
        return cls.get_link_name(0)

    @classmethod
    def get_link_name(cls, n: int) -> str:
        return cls.__construct_name(f'{cls.__link_basename}{n}')

    @classmethod
    def get_joint_name(cls, n: int) -> str:
        """
        :param n: index of the link that is controlled by the joint
        :return: name of the joint
        """
        links = [f'{cls.__link_basename}{_}' for _ in (n, n + 1)]
        return cls.__construct_name(cls.__joint_basename, *links)

    @classmethod
    def get_tip_name(cls):
        return cls.__construct_name('tip')

    @classmethod
    def get_sensor_name(cls, type: str) -> str:
        return cls.__construct_name(cls.__sensor_basename, type)

    def __init_handles(self):
        self.__base_handle, *self.__links_handles = [
                vrepapi.simxGetObjectHandle(self.client_id, self.get_link_name(_), vrepapi.simx_opmode_blocking)[1]
                for _ in range(self.__n_links)]
        self.__joint_handles = [
                vrepapi.simxGetObjectHandle(self.client_id, self.get_joint_name(_), vrepapi.simx_opmode_blocking)[1] for
                _ in range(0, self.__n_links - 1)]
        _, self.__tip_handle = vrepapi.simxGetObjectHandle(self.client_id, self.get_tip_name(),
                                                           vrepapi.simx_opmode_blocking)
        _, self.__prox_sensor_handle = vrepapi.simxGetObjectHandle(self.client_id, self.get_sensor_name('proximity'),
                                                                   vrepapi.simx_opmode_blocking)
        _, self.__vis_sensor_handle = vrepapi.simxGetObjectHandle(self.client_id, self.get_sensor_name('visual'),
                                                                  vrepapi.simx_opmode_blocking)

    def __init_datastreams(self):
        vrepapi.simxReadProximitySensor(self.client_id, self.__prox_sensor_handle, vrepapi.simx_opmode_streaming)
        vrepapi.simxReadVisionSensor(self.client_id, self.__vis_sensor_handle, vrepapi.simx_opmode_streaming)

    def __init(self):
        self.__init_handles()
        self.__init_datastreams()

    @property
    def distance(self) -> Optional[float]:
        print(f'Reading from proximity sensor...')

        res, ready, (_, _, d), *_ = vrepapi.simxReadProximitySensor(self.client_id, self.__prox_sensor_handle,
                                                                    vrepapi.simx_opmode_buffer)

        print(f'Readings:\n'
              f'  response: {res}; ready: {ready}; distance: {d}\n')

        return d if self.response_good(res) and ready else None

    @property
    def greenness(self):
        print(f'Reading from visual sensor...')

        res, ready, packets = vrepapi.simxReadVisionSensor(self.client_id, self.__vis_sensor_handle,
                                                           vrepapi.simx_opmode_buffer)

        greenness = packets[0][12] if self.response_good(res) and packets else None  # get average green channel

        print(f'Readings:\n'
              f'  response: {res}; ready: {ready}; greenness: {packets}\n')

        return greenness

    @property
    def object_found(self):
        return self.greenness and self.greenness >= self.__G_THRESH

    def __move_to_next_point(self):
        self.move_joint(self.__joint_handles[0], self.__joint_dp[0])
        self.set_joint_pos(self.__joint_handles[1], pi/4)
        min_, max_ = self.__joint_ranges[-1]
        if abs((cur_pos := self.get_joint_pos(self.__joint_handles[-1])) - max_) <= self.__ANGLE_EPS:
            self.__joint_dp[-1] = -abs(self.__joint_dp[-1])
        elif abs(cur_pos - min_) <= self.__ANGLE_EPS:
            self.__joint_dp[-1] = abs(self.__joint_dp[-1])
        self.move_joint(self.__joint_handles[-1], self.__joint_dp[-1])

    def get_joint_pos(self, joint_handle: VREPHandle):
        _, curr_coord = vrepapi.simxGetJointPosition(self.client_id, joint_handle, vrepapi.simx_opmode_blocking)
        print(f'Getting joint pos... pos: {curr_coord}')
        return curr_coord if self.response_good(_) else None

    def move_joint(self, joint_handle: VREPHandle, delta: float):
        curr_pos = self.get_joint_pos(joint_handle)
        if curr_pos is None:
            return
        self.set_joint_pos(joint_handle, curr_pos + delta)

    def set_joint_pos(self, joint_handle: VREPHandle, pos: float):
        print(f'Setting joint pos: {pos}')
        vrepapi.simxSetJointTargetPosition(self.client_id, joint_handle, pos, vrepapi.simx_opmode_oneshot)

    def set_joint_vel(self, joint_handle: VREPHandle, vel: float):
        vrepapi.simxSetJointTargetVelocity(self.client_id, joint_handle, vel, vrepapi.simx_opmode_oneshot)

    def __touch_object(self):
        while True:
            if self.distance is not None and self.distance <= self.__D_EPS:
                break
            self.move_joint(self.__joint_handles[1], 0.001)
            self.move_joint(self.__joint_handles[2], -0.001)

    def __home(self):
        for joint_handle in self.__joint_handles:
            self.set_joint_pos(joint_handle, 0)
        while not all(isclose(self.get_joint_pos(_), 0, abs_tol=self.__D_EPS) for _ in self.__joint_handles):
            pass

    def _run(self):
        self.__init()
        while not self.object_found:
            self.__move_to_next_point()
        self.__touch_object()
        self.__home()


if __name__ == '__main__':
    SERVER_HOST = '127.0.0.1'
    SERVER_PORT = 19999

    controller = L3Controller(SERVER_HOST, SERVER_PORT)

    controller.run()
