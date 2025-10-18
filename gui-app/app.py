import streamlit as st
from melfapy.Melfa import *
import math

st.title("Melfapy App")
col1, col2, col3, col4 = st.columns(4)

with col1:
    x = st.text_input("x")
with col2:
    y = st.text_input("y")
with col3:
    z = st.text_input("z")
with col4:
    angle = st.text_input("angle")
    math.radians(float(angle))

st.write("x:", x, "y:", y, "z:", z)

btn = col1.button("Send Coodinates")
pose = [x, y, z, 0, 0, angle, 0, 0, 4, 0]
datalink = MelfaDatalink(pose)

print(datalink)
if btn:
    st.write("Send Coodinates")
    datalink.listen(address=("192.168.0.10", 10009))
    btn = False

