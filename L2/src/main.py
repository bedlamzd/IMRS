from abc import ABCMeta, abstractmethod
from typing import Any, Optional

import vrepapi.sim as vrepapi


class VREPClient(metaclass=ABCMeta):
    client_id: Any
    __host: Any
    __port: Any

    def __init__(self, host, port):
        self.__host = host
        self.__port = port

    def __enter__(self):
        print(f'Initiating connection with remote "{self.__host}:{self.__port}"...')
        self.client_id = vrepapi.simxStart(self.__host, self.__port, True, True, 5000, 5)
        print(f'Connection initiated. CLIENT_ID: {self.client_id}')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print(exc_type, exc_val, exc_tb)
        print(f'Closing remote connection with client {self.client_id}')
        vrepapi.simxFinish(self.client_id)

    @abstractmethod
    def _run(self):
        pass

    @staticmethod
    def response_good(code: int) -> bool:
        return code in (vrepapi.simx_return_ok, vrepapi.simx_return_novalue_flag)

    def run(self):
        with self:
            self._run()


class L2Controller(VREPClient):
    LEFT_HOLDER_NAME: str = 'left_holder_joint'
    RIGHT_HOLDER_NAME: str = 'right_holder_joint'
    SENSOR_NAME: str = 'left_holder_sensor'

    D_EPS = 1e-4

    def __init_handlers(self):
        # Get handles to the objects
        print('Initiating handlers for objects')
        _, self.left_holder = vrepapi.simxGetObjectHandle(
                self.client_id, self.LEFT_HOLDER_NAME,
                vrepapi.simx_opmode_blocking
                )
        _, self.right_holder = vrepapi.simxGetObjectHandle(
                self.client_id, self.RIGHT_HOLDER_NAME,
                vrepapi.simx_opmode_blocking
                )
        _, self.sensor = vrepapi.simxGetObjectHandle(
                self.client_id, self.SENSOR_NAME,
                vrepapi.simx_opmode_blocking
                )

    def __init_datastreams(self):
        print('Initiating data streams')
        # initialize stream to get distance
        vrepapi.simxReadProximitySensor(self.client_id, self.sensor, vrepapi.simx_opmode_streaming)

    @property
    def distance(self) -> Optional[float]:
        print(f'Reading from sensor...')

        res, ready, (_, _, d), *_ = vrepapi.simxReadProximitySensor(
                self.client_id, self.sensor, vrepapi.simx_opmode_buffer
                )

        print(f'Readings:'
              f'\n  response: {res}; ready: {ready}; distance: {d}\n')

        if self.response_good(res) and ready:
            if self.initial_dist is None:
                self.initial_dist = d
            return d

        return None

    @property
    def sim_time(self) -> float:
        res, ints, floats, strings, buffer = \
            vrepapi.simxCallScriptFunction(
                    self.client_id,
                    'Gripper',
                    vrepapi.sim_scripttype_childscript,
                    'getSimTime_function',
                    [], [], [], bytearray(),
                    vrepapi.simx_opmode_blocking
                    )
        return floats[0] if self.response_good(res) else -1

    @property
    def initial_dist(self):
        try:
            return self.__initial_dist
        except AttributeError:
            return None

    @initial_dist.setter
    def initial_dist(self, val: float):
        self.__initial_dist = val

    def __set_gripper_velocity(self, vel: float):
        vrepapi.simxSetJointTargetVelocity(self.client_id, self.left_holder, vel, vrepapi.simx_opmode_oneshot)
        vrepapi.simxSetJointTargetVelocity(self.client_id, self.right_holder, vel, vrepapi.simx_opmode_oneshot)

    def __clench(self, vel: float):
        print(f'Clenching gripper with velocity: {vel}')
        while True:
            if self.distance is not None and self.distance <= self.D_EPS:
                break
            self.__set_gripper_velocity(abs(vel))
        self.__set_gripper_velocity(0)

    def __wait(self, n: int):
        print(f'Waiting for {n} sec (sim time)...')
        start = self.sim_time
        while self.sim_time - start < n:
            pass

    def __unclench(self, vel: float):
        print(f'Unclenching gripper with velocity: {vel}')
        while abs(self.distance - self.initial_dist) > self.D_EPS:
            self.__set_gripper_velocity(-abs(vel))
        self.__set_gripper_velocity(0)

    def _run(self):
        self.__init_handlers()
        self.__init_datastreams()
        self.__clench(0.008)
        self.__wait(5)
        self.__unclench(0.008)


if __name__ == '__main__':
    SERVER_HOST = '127.0.0.1'
    SERVER_PORT = 19999

    controller = L2Controller(SERVER_HOST, SERVER_PORT)

    controller.run()
