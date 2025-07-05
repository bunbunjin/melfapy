from src.melfapy.Melfa import *

pose = MelfaPose([350, 100, 100, 0, 0, 0, 0, 0, 0, 0])
s = MelfaPacket(
    command=1,
    send_type=0,
    recv_type=0,
    pose=pose
)
s.send_packet()
