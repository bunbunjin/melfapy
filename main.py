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
    address=("192.168.0.10", 10000),
)
sample.send_packet()


j1 = 100
j2 = 200
j3 = 300
sample_joint = MelfaPose([j1, j2, j3, 0, 0, 0, 0, 0])
sample = MelfaController(
    command=1,
    send_type=2,
    recv_type=1,
    pose=sample_joint,
    address=("192.168.0.10", 10000),
)
sample.send_packet()


sample_datalink = [200, 100, 100, 0, 0, 0, 0, 0, 4, 0]
sample = MelfaDatalink(sample_datalink)
sample.listen(address=("192.168.0.20", 10009))
