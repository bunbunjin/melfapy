# melpy

## Overview
This is the Python library for controlling Mitsubishi MELFA .

1. Run SAMPLE.prg in ToolBox3.
2. Specify the target position with MelafaPose 
3. Use MelfaPacket to set the command, transmission data type, response data type, and position data
4. Start the robot motion by calling the send_packet() method

> MELFA Coordinate Format <br>
> [x, y, z, a, b, c f1, f2] <br>
> [j1, j2, j3, j4, j5, j6, j7, j8]

---

# Command Definitions
The arguments of MelfaController() are explained below.<br>
Basically, select 1.<br>
In the current implementation, SendType can only use pos '1' and jnt '2'.

### Command

Specifies whether to enable or disable real-time external commands, or to terminate them.<br>

* `0` // No real-time external command<br>
* `1` // Real-time external command enabled<br>
* `255` // Terminate real-time external command

---

### SendType

① When sending from PC to controller (command mode):<br>
Specifies the type of position data sent from the PC.<br>
For the first transmission, set to “no data.”<br>

* `0` // No data<br>
* `1` // Cartesian data<br>
* `2` // Joint data<br>
* `3` // Motor pulse data<br>

② When receiving from controller to PC (monitor mode):<br>
Indicates the type of position data returned by the controller.<br>

* `0` // No data<br>
* `1` // Cartesian data<br>
* `2` // Joint data<br>
* `3` // Motor pulse data<br>
* `4` // Cartesian data (after filter processing)<br>
* `5` // Joint data (after filter processing)<br>
* `6` // Motor pulse data (after filter processing)<br>
* `7` // Cartesian data (encoder feedback)<br>
* `8` // Joint data (encoder feedback)<br>
* `9` // Motor pulse data (encoder feedback)<br>
* `10` // Current command \[%]<br>
* `11` // Current feedback \[%]<br>

※ Same as `RecvType`. Either can be used.

---

### RecvType

① When sending from PC to controller (command mode):<br>
Specifies the type of data to be returned from the controller.<br>

* `0` // No data<br>
* `1` // Cartesian data<br>
* `2` // Joint data<br>
* `3` // Pulse data<br>
* `4` // Cartesian data (after filter processing)<br>
* `5` // Joint data (after filter processing)<br>
* `6` // Motor pulse data (after filter processing)<br>
* `7` // Cartesian data (encoder feedback)<br>
* `8` // Joint data (encoder feedback)<br>
* `9` // Motor pulse data (encoder feedback)<br>
* `10` // Current command \[%]<br>
* `11` // Current feedback \[%]<br>

② When receiving from controller to PC (monitor mode):<br>
Indicates the type of position data returned by the controller.<br>
(Same codes as above.)<br>

※ Same as `SendType`. Either can be used.

---

### Position Data (pos / jnt / pls)

① When sending from PC to controller (command mode):<br>
Specifies the target position data. Use the format defined by `SendType`.<br>

② When receiving from controller to PC (monitor mode):<br>
Indicates the position data returned by the controller.<br>
The data type is defined by `SendType (= RecvType)`.<br>

* `POSE`  // Cartesian type \[mm/rad]<br>
* `JOINT` // Joint type \[rad]<br>
* `PULSE` // Motor pulse type \[pulse] or current type \[%]<br>

---

### SendIOType

① When sending from PC to controller (command mode):<br>
Specifies the type of I/O signal data to send. Use “no data” if unused.<br>

② When receiving from controller to PC (monitor mode):<br>
Indicates the type of I/O signal data returned by the controller.<br>

* `0` // No data<br>
* `1` // Output signal<br>
* `2` // Input signal<br>

---

### RecvIOType

① When sending from PC to controller (command mode):<br>
Specifies the type of I/O signal data to be returned by the controller.<br>
Use “no data” if unused.<br>

* `0` // No data<br>
* `1` // Output signal<br>
* `2` // Input signal<br>

② When receiving from controller to PC (monitor mode):<br>
Not used.<br>

---

### I/O Signal Data (BitTop, BitMask, IoData)

① When sending from PC to controller (command mode):<br>
Specifies the output signal data to send.<br>

② When receiving from controller to PC (monitor mode):<br>
Indicates the I/O signal data returned by the controller.<br>

* `BitTop`  // Starting bit number of input or output signal<br>
* `BitMask` // Bitmask pattern (valid only in command mode)<br>
* `IoData`  // Value of input/output signal (monitor mode) or output value (command mode)<br>

---

### Tcount (Timeout Counter)

① When sending from PC to controller (command mode):<br>
Not used.<br>

② When receiving from controller to PC (monitor mode):<br>
Indicates the number of cycles without communication, when the timeout parameter `MXTTOUT` is not -1.<br>
Resets to 0 at the start of an MXT command.<br>

---

### Ccount (Communication Counter)

Used both when sending (command mode) and receiving (monitor mode).<br>

---

## Troubleshooting
For speed-related issues: adjust the values of _v_max, _a_max, and _j_max.<br>
For range-related issues: check the arm position in ToolBox3.<br>
Others: Most problems can be solved by asking ChatGPT.
