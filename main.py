from src.melfapy.Melfa import *

x = 200
y = 100
sample_pose = MelfaPose([x, y, 100, 0, 0, 29, 0, 0, 4, 0])
sample = MelfaController(
    command=1,
    send_type=1,
    recv_type=1,
    pose=sample_pose,
    j_max=400,
    a_max=500,
    v_max=500,
    address=('192.168.0.10', 10000)
)
sample.send_packet()
