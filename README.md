# UDP Motion Capture Demo for Optitrack Motive NatNet messages
This demo is able capture NatNet messages and outputs the containing rigid objects to the bash.
### Tested
At the moment is is only tested with 
- Motive 1.10.1.

## Source / Thanks 
The code is based on the mocap lib (https://github.com/clearpathrobotics/mocap_optitrack.git)
Thanks to all authores
 
## Setup
### Motive (server)
- View -> Data Streaming
 - Set Network Interface
 - Streaming Settings -> set all Stream x exept Rigid Bodies to false
- Advanced Network Settings
 - Command Port: 1510
 - Data Port: 9000
 - Multicast Interface: IP of Client
### NatNet (client)
The settings are currently hardcoded in mocap_optitrack/data_model.h
 - Command Port: 1510
 - Data Port: 9000
 - Multicast Ip Address: 224.0.0.1

## Problems
if the Multicast IP Address is wrong you will get the following error

```
terminate called after throwing an instance of 'SocketException'
  what():  Failed to set socket option: EINVAL
*** Failure: /home/max/projects/OptiTrack/build/optitrack has crashed ***
```
