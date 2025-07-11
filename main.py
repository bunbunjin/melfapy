from src.melfapy.Melfa import *

sample_pose = MelfaPose([130, 100, 100, 0, 0, 0, 0, 0, 0, 0])
sample = MelfaPacket(
    command=1,
    send_type=0,
    recv_type=0,
    pose=sample_pose
)
sample.send_packet()
