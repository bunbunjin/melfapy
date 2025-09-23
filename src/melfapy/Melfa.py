import asyncio
import concurrent.futures
import threading
import dataclasses
from dataclasses import dataclass, field
from .utils.advanced_S_curve_acceleration import AdvancedSCurvePlanner
import struct
import socket
import time
import numpy as np
import math


@dataclass
class MelfaPose:
    """
    args:
        values: If you use position coordinate, this argument [x, y, z, a, b, c, l1, l2]
                If you use joint coordinates, this argument [j1, j2, j3, j4, j5, j6, j7, j8].
    """

    values: list

    def __getitem__(self, item):
        return self.values[item]

    def as_floats(self) -> list:
        pose = [int(v) if i > 7 else float(v) for i, v in enumerate(self.values)]
        return pose

    def as_comma(self) -> str:
        pose = ",".join(map(str, self.values)) + "\r\n"
        return pose


@dataclass
class MelfaPacket:
    command: int
    send_type: int
    recv_type: int
    pose: MelfaPose
    send_io_type: int = 0
    recv_io_type: int = 0
    bit_top: int = 0
    bit_mask: int = 0
    io_data: int = 0
    tcount: int = 0
    ccount: int = 1
    ex_pose: MelfaPose = field(default_factory=list)
    address: tuple[str, int] = ("192.168.0.20", 10001)
    lock = asyncio.Lock()
    state = [0, 0, 0, 0, 0, 0, 0, 0, 4, 0]
    done_flags = [False, False, False, False]

    def to_bytes(self) -> bytes:
        reserve = 0
        reserve_type = 0
        fmt = "<HHHHffffffffIIHHHHHHLHHffffffffIIHHffffffffIIHHffffffffII"
        args = [
            self.command,  # H
            self.send_type,  # H
            self.recv_type,  # H
            reserve,  # H
            *self.pose.as_floats(),  # ffffffffII
            self.send_io_type,  # H
            self.recv_io_type,  # H
            self.bit_top,  # H
            self.bit_mask,  # H
            self.io_data,  # H
            self.tcount,  # H
            self.ccount,  # L
            reserve,  # H
            reserve_type,  # H
            *self.ex_pose.as_floats(),
            reserve,
            reserve_type,
            *self.ex_pose.as_floats(),
            reserve,
            reserve_type,
            *self.ex_pose.as_floats(),
        ]

        return struct.pack(fmt, *args)


@dataclass
class MelfaController(MelfaPacket):
    v_max: int = 300  # Max speed
    a_max: int = 500  # Max acceleration
    j_max: int = 700  # # Max jark
    sleep_time = 0.0031

    def get_position(self) -> tuple:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(self.address)
            _zero_pose = MelfaPose([0] * 10)
            _chack_positon_packet = MelfaPacket(
                command=0,
                send_type=0,
                recv_type=1,
                pose=_zero_pose,
                ccount=1,
                ex_pose=_zero_pose,
                send_io_type=0,
                recv_io_type=0,
            )

            data = _chack_positon_packet.to_bytes()
            s.sendto(data, self.address)
            time.sleep(0.0071)
            data = s.recv(1024)

            recv_data = struct.unpack(
                "<HHHHffffffffIIHHHHHHLHHffffffffIIHHffffffffIIHHffffffffII", data
            )
            position = recv_data[4:14]
            print(f"positon = {position}")
            return position

    async def run_axis(self, name, curve, dt, total_time) -> None:
        axis_index = {"x": 0, "y": 1, "z": 2, "angle": 5}
        for t in np.arange(0, total_time + 1, self.sleep_time):
            pos, vel, acc, jerk = curve.get_profile(t)
            pos = float(pos)
            async with self.lock:
                self.state[axis_index[name]] = pos
            t += dt
            await asyncio.sleep(dt)

        async with self.lock:
            if name == "angle":
                axis_index["angle"] = 3
            self.done_flags[axis_index[name]] = True
            print(
                f"{name}完了: x={self.state[0]:.3f}, y={self.state[1]:.3f}, z={self.state[2]:.3f}"
            )

    async def get_current_pose(self) -> list:
        async with self.lock:
            return list(self.state)

    async def send_pose(self, s) -> None:
        while True:
            async with self.lock:
                if all(self.done_flags):
                    break
            stream_pose = self.state

            print(f"Send to coordinate for Melfa: {stream_pose}")
            await asyncio.sleep(self.sleep_time)
            packet = MelfaPacket(
                command=self.command,
                send_type=self.send_type,
                recv_type=self.recv_type,
                ex_pose=MelfaPose([0] * 10),
                pose=MelfaPose(stream_pose),
            )
            s.sendto(packet.to_bytes(), self.address)

    def send_packet(self) -> None:
        _POSE = self.pose
        _zero_pose = MelfaPose([0] * 10)
        _first_packet = MelfaPacket(
            command=0,
            send_type=0,
            recv_type=0,
            pose=_zero_pose,
            ccount=1,
            ex_pose=_zero_pose,
            send_io_type=0,
            recv_io_type=0,
        )
        _end_packet = MelfaPacket(
            command=255,
            send_type=1,
            recv_type=1,
            pose=_zero_pose,
            ccount=1,
            ex_pose=_zero_pose,
        )

        x = _POSE[0]
        y = _POSE[1]
        z = _POSE[2]
        angle = math.radians(_POSE[5])

        _init_POSE = self.get_position()
        _init_x = _init_POSE[0]
        _init_y = _init_POSE[1]
        _init_z = _init_POSE[2]
        _init_angle = _init_POSE[5]
        print("send to coordinate for Melfa")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(self.address)
            data = _first_packet.to_bytes()
            s.sendto(data, self.address)
            print("[INFO] Send to First packet", "-" * 10)
            _q0 = _init_x
            _q1 = x
            x_curve = AdvancedSCurvePlanner(
                _q0, _q1, self.v_max, self.a_max, self.j_max
            )

            _q0 = _init_y
            _q1 = y
            y_curve = AdvancedSCurvePlanner(
                _q0, _q1, self.v_max, self.a_max, self.j_max
            )

            _q0 = _init_z
            _q1 = z
            z_curve = AdvancedSCurvePlanner(
                _q0, _q1, self.v_max, self.a_max, self.j_max
            )

            _q0 = _init_angle
            _q1 = angle

            a_curve = AdvancedSCurvePlanner(
                _q0, _q1, self.v_max, self.a_max, self.j_max
            )

            _x_total_time = x_curve.T
            _y_total_time = y_curve.T
            _z_total_time = z_curve.T
            _a_total_time = a_curve.T

            time.sleep(self.sleep_time)

            async def position_publish():
                move_x, move_y, move_z, move_a, pos = await asyncio.gather(
                    self.run_axis(
                        name="x",
                        curve=x_curve,
                        total_time=_x_total_time,
                        dt=self.sleep_time,
                    ),
                    self.run_axis(
                        name="y",
                        curve=y_curve,
                        total_time=_y_total_time,
                        dt=self.sleep_time,
                    ),
                    self.run_axis(
                        name="z",
                        curve=z_curve,
                        total_time=_z_total_time,
                        dt=self.sleep_time,
                    ),
                    self.run_axis(
                        name="angle",
                        curve=a_curve,
                        total_time=_a_total_time,
                        dt=self.sleep_time,
                    ),
                    self.send_pose(s),
                )

            asyncio.run(position_publish())

        return None


@dataclasses.dataclass
class MelfaDatalink(MelfaPose):
    def listen(self, address: tuple[str, int] = ("192.168.0.20", 10009)) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(address)
            pose = self.as_comma()
            print(pose.encode("ascii"))
            s.sendall(pose.encode("ascii"))

    def confirm_pose(self):
        pose = self.as_comma()
        print(pose.encode("ascii"))
