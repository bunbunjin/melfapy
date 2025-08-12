from src.melfapy.Melfa import *

x = 200
y = 100
sample_pose = MelfaPose([x, y, 100, 0, 0, 29, 0, 0, 4, 0])
sample = MelfaPacket(
    command=1,
    send_type=1,
    recv_type=1,
    pose=sample_pose
)
sample.send_packet()
