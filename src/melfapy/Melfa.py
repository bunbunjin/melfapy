from dataclasses import dataclass, field
from utils.advanced_S_curve_acceleration import AdvancedSCurvePlanner

# import trio
import struct
import socket
import time
import numpy as np
import math


@dataclass
class MelfaPose:
    values: list  # 座標（10つのfloat）

    def __getitem__(self, item):
        return self.values[item]

    def as_floats(self) -> list:
        pose = [int(v) if i > 7 else float(v) for i, v in enumerate(self.values)]
        return pose


@dataclass
class MelfaIO:
    bit_top: int = 0
    bit_mask: int = 0
    io_data: int = 0


@dataclass
class MelfaPacket:

    command: int
    send_type: int
    recv_type: int
    pose: MelfaPose
    send_io_type: int = 0
    recv_io_type: int = 0
    io: MelfaIO = field(default_factory=MelfaIO)
    tcount: int = 0
    ccount: int = 1
    ex_pose: MelfaPose = MelfaPose([0]*10)
    address: tuple[str, int] = ("192.168.0.20", 10000)

    """
        @property
        def address(self):
            return self._address

        @address.setter
        def address(self, value: tuple[str, int]):
            if not value:
                raise ValueError("MelfaPacket.address is empty")
            self._address = value
    """

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
            self.io.bit_top,  # H
            self.io.bit_mask,  # H
            self.io.io_data,  # H
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

            return position

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
        angle = _POSE[3]

        _init_POSE = self.get_position()
        _init_x = _init_POSE[0]
        _init_y = _init_POSE[1]
        _init_z = _init_POSE[2]
        _init_angle = _init_POSE[3]

        _q1 = _POSE[0]  # 終了位置
        _v_max = 20  # 最大速度
        _a_max = 10  # 最大加速度
        _j_max = 50  # 最大ジャーク
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(self.address)
            data = _first_packet.to_bytes()
            s.sendto(data, self._address)
            print("[INFO] Send to First packet\n", "-" * 10)
            _q0 = _init_x  
            _q1 = x
            x_curve = AdvancedSCurvePlanner(_q0, _q1, _v_max, _a_max, _j_max)
            _q0 = _init_y
            _q1 = y
            y_curve = AdvancedSCurvePlanner(_q0, _q1, _v_max, _a_max, _j_max)
            _q0 = _init_z
            _q1 = z
            z_curve = AdvancedSCurvePlanner(_q0, _q1, _v_max, _a_max, _j_max)
            _q0 = _init_angle
            _q1 = angle
            a_curve = AdvancedSCurvePlanner(_q0, _q1, _v_max, _a_max, _j_max)
            _x_total_time = x_curve.T
            _y_total_time = y_curve.T
            _z_total_time = z_curve.T
            _a_total_time = a_curve.T
            s.sendto(data, self.address)
            time.sleep(0.0071)
            print("-" * 10, "[INFO] X axis Phase", "-" * 10)
            for t in np.arange(0, _x_total_time, 0.0071):
                pos, vel, acc, jerk = x_curve.get_profile(t)
                x = float(pos)

                _x_pose = MelfaPose(
                    [x, _init_y, _init_z, 0, 0, _init_angle, 0, 0, 0, 0]
                )
                packet = MelfaPacket(
                    command=self.command,
                    send_type=self.send_type,
                    recv_type=self.recv_type,
                    pose=_x_pose,
                    ccount=self.ccount,
                    ex_pose=_zero_pose,
                )

                s.sendto(packet.to_bytes(), self.address)
                time.sleep(0.0071)

            print("-" * 10, "[INFO] Y axis Phase", "-" * 10)
            time.sleep(0.0071)
            for t in np.arange(0, _y_total_time, 0.0071):
                pos, vel, acc, jerk = y_curve.get_profile(t)
                y = float(pos)
                _y_pose = MelfaPose([x, y, _init_z, 0, 0, _init_angle, 0, 0, 0, 0])
                packet = MelfaPacket(
                    command=self.command,
                    send_type=self.send_type,
                    recv_type=self.recv_type,
                    pose=_y_pose,
                    ccount=self.ccount,
                    ex_pose=_zero_pose,
                )

                s.sendto(packet.to_bytes(), self.address)
                time.sleep(0.0071)
            print("-" * 10, "[INFO] Z axis Phase", "-" * 10)

            for t in np.arange(0, _z_total_time, 0.0071):
                pos, vel, acc, jerk = z_curve.get_profile(t)
                z = float(pos)

                _z_pose = MelfaPose([x, y, z, 0, 0, _init_angle, 0, 0, 0, 0])
                packet = MelfaPacket(
                    command=self.command,
                    send_type=self.send_type,
                    recv_type=self.recv_type,
                    pose=_z_pose,
                    ccount=self.ccount,
                    ex_pose=_zero_pose,
                )
                s.sendto(packet.to_bytes(), self.address)
            print("-" * 10, "[INFO] Angle axis Phase", "-" * 10)
            for t in np.arange(0, _a_total_time, 0.0071):
                pos, vel, acc, jerk = x_curve.get_profile(t)

                angle = pos
                _angle_pose = MelfaPose([x, y, z, 0, 0, angle, 0, 0, 0, 0])
                packet = MelfaPacket(
                    command=self.command,
                    send_type=self.send_type,
                    recv_type=self.recv_type,
                    pose=_angle_pose,
                    ccount=self.ccount,
                    ex_pose=_zero_pose,
                )
                s.sendto(packet.to_bytes(), self._address)
        return None
