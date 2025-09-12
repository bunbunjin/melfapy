# melpy

## Overview
This is the Python library for controlling Mitsubishi MELFA .

1. Run SAMPLE.prg in ToolBox3.
2. Specify the target position with MelafaPose 
3. Use MelfaPacket to set the command, transmission data type, response data type, and position data
4. Start the robot motion by calling the send_packet() method

> MELFA Coordinate Format 
> [x, y, z, a, b, c f1, f2]

## Troubleshooting
For speed-related issues: adjust the values of _v_max, _a_max, and _j_max.<br>
For range-related issues: check the arm position in ToolBox3.<br>
Others: Most problems can be solved by asking ChatGPT.
