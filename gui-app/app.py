import streamlit as st
from melfapy.Melfa import *


st.title("Melfapy App")
col1, col2, col3 = st.columns(3)

with col1:
    x = st.text_input("x")
with col2:
    y = st.text_input("y")
with col3:
    z = st.text_input("z")

st.write("x:", x, "y:", y, "z:", z)
btn = col1.button("Send Coodinates")
pose = [x, y, z, 0, 0, 0, 0, 0, 4, 0]
datalink = MelfaDatalink(pose)
print(datalink)
if btn:
    st.write("Send Coodinates")
    datalink.listen(address=("192.168.0.10", 10009))
    btn = False

