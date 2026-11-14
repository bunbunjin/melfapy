from pydantic import BaseModel, Field, PrivateAttr
from typing import ClassVar
import asyncio
import concurrent.futures
import threading
import dataclasses
from dataclasses import dataclass
from .utils.advanced_S_curve_acceleration import AdvancedSCurvePlanner
from .utils.trajectory import TrajectorySample
import struct
import socket
import time
import numpy as np
import math


MXT_FORMAT = "<HHHHffffffffIIHHHHHHLHHffffffffIIHHffffffffIIHHffffffffII"


class MelfaPose(BaseModel):
    """
    args:
        values: If you use position coordinate, this argument [x, y, z, a, b, c, l1, l2]
                If you use joint coordinates, this argument [j1, j2, j3, j4, j5, j6, j7, j8].
    """

    values: list[float]

    def __getitem__(self, item):
        return self.values[item]

    def as_floats(self) -> list:
        pose = [int(v) if i > 7 else float(v) for i, v in enumerate(self.values)]
        return pose

    def as_comma(self) -> str:
        print(self.values)
        for i in self.values:
            print(i, type(i))
        pose = ",".join(map(str, self.values)) + "\r\n"
        return pose


class MelfaPacket(BaseModel):
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
    ex_pose: MelfaPose = Field(default_factory=lambda: MelfaPose(values=[0] * 10))
    address: tuple[str, int] = ("192.168.0.20", 10001)
    lock: ClassVar[asyncio.Lock] = asyncio.Lock()
    state: list[int] = [0, 0, 0, 0, 0, 0, 0, 0, 4, 0]
    done_flags: list[bool] = [False, False, False, False]

    def to_bytes(self) -> bytes:
        reserve = 0
        reserve_type = 0
        fmt = MXT_FORMAT
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


class MelfaController(MelfaPacket):
    v_max: int = 300  # Max speed
    a_max: int = 500  # Max acceleration
    j_max: int = 700  # # Max jark
    sleep_time: float = 0.0031  # Minimum interval between command packets [s]
    reply_timeout: float = 1.0  # Max time to wait for a reply from the controller [s]
    _trajectory: list[TrajectorySample] = PrivateAttr(default_factory=list)

    @property
    def trajectory(self) -> list[TrajectorySample]:
        """Commands sent and replies received during the last send_packet()."""
        return list(self._trajectory)

    def _open_socket(self) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(self.reply_timeout)
        return s

    def _exchange(self, s: socket.socket, packet: MelfaPacket) -> tuple:
        """Send one packet and wait for the controller's reply to it.

        The controller answers every MXT packet. Reading that reply before
        sending the next packet (or closing the socket) keeps the PC in step
        with the controller's control cycle.
        """
        s.sendto(packet.to_bytes(), self.address)
        try:
            data, _ = s.recvfrom(1024)
        except socket.timeout:
            raise TimeoutError(
                f"No reply from MELFA controller {self.address} "
                f"within {self.reply_timeout} s"
            ) from None
        return struct.unpack(MXT_FORMAT, data)

    @staticmethod
    def _wait_until(deadline: float) -> None:
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return
            if remaining > 0.002:
                time.sleep(remaining - 0.002)

    def get_position(self) -> tuple:
        _zero_pose = MelfaPose(values=[0] * 10)
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
        with self._open_socket() as s:
            recv_data = self._exchange(s, _chack_positon_packet)

        position = recv_data[4:14]
        print(f"positon = {position}")
        return position

    def send_packet(self) -> None:
        _POSE = self.pose
        _zero_pose = MelfaPose(values=[0] * 10)
        _first_packet = MelfaPacket(
            command=0,
            send_type=0,
            recv_type=1,
            pose=_zero_pose,
            ccount=1,
            ex_pose=_zero_pose,
            send_io_type=0,
            recv_io_type=0,
        )

        x = _POSE[0]
        y = _POSE[1]
        z = _POSE[2]
        angle = math.radians(_POSE[5])

        print("send to coordinate for Melfa")
        with self._open_socket() as s:
            # The reply to the first packet carries the current position.
            _init_POSE = list(self._exchange(s, _first_packet)[4:14])
            print("[INFO] Send to First packet", "-" * 10)

            # index in the pose -> motion profile from current to target value
            curves = {
                index: AdvancedSCurvePlanner(
                    _init_POSE[index], target, self.v_max, self.a_max, self.j_max
                )
                for index, target in ((0, x), (1, y), (2, z), (5, angle))
            }
            total_time = max(curve.T for curve in curves.values())

            # Axes without a profile keep the value reported by the controller.
            stream_pose = list(_init_POSE)
            self._trajectory = []
            start = time.perf_counter()
            next_send = start
            while True:
                # The profile is sampled at the real elapsed time, so the
                # motion takes total_time however fast the loop runs.
                t = time.perf_counter() - start
                finished = t >= total_time
                for index, curve in curves.items():
                    stream_pose[index] = float(curve.get_profile(t)[0])

                packet = MelfaPacket(
                    command=self.command,
                    send_type=self.send_type,
                    recv_type=self.recv_type,
                    ex_pose=_zero_pose,
                    pose=MelfaPose(values=stream_pose),
                )
                reply = self._exchange(s, packet)
                self._trajectory.append(
                    TrajectorySample(
                        t_command=t,
                        t_reply=time.perf_counter() - start,
                        command=tuple(stream_pose[:8]),
                        reply_type=reply[1],
                        reply=tuple(reply[4:12]),
                        ccount=reply[20],
                    )
                )
                if finished:
                    break

                next_send = max(next_send + self.sleep_time, time.perf_counter())
                self._wait_until(next_send)

        return None

    def send_raw_xy(self, pose: list[list[float]], x_offset: float=0, y_offset:float=0) -> None:
        _zero_pose = MelfaPose(values=[0] * 10)
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

        with self._open_socket() as s:
            self._exchange(s, _first_packet)
            next_send = time.perf_counter()
            for i in pose:
                x, y = i[0], i[1]
                xy_pose = MelfaPose(
                    values=[x + x_offset, y + y_offset, 100, 0, 0, 29, 0, 0, 4, 0]
                )

                raw_xy_packet = MelfaPacket(
                    command=0,
                    send_type=0,
                    recv_type=0,
                    pose=xy_pose,
                    ccount=1,
                    ex_pose=_zero_pose,
                    send_io_type=0,
                    recv_io_type=0,
                )
                next_send = max(next_send + self.sleep_time, time.perf_counter())
                self._wait_until(next_send)
                self._exchange(s, raw_xy_packet)


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
