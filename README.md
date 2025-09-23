# melfapy

## Overview
This is the Python library for controlling Mitsubishi MELFA .

1. Run SAMPLE.prg in ToolBox3.
2. Specify the target position with MelfaPose 
3. Use MelfaPacket to set the command, transmission data type, response data type, and position data
4. Start the robot motion by calling the send_packet() method

## MELFA Coordinate Format <br>
> [x, y, z, a, b, c f1, f2] <br>
> [j1, j2, j3, j4, j5, j6, j7, j8]

## Documentation
- [Using MXT](docs/MXT.md)
- [Using Datalink](docs/datalink.md)