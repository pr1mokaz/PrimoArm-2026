#!/usr/bin/env python3
# module: ping_ax12.py


from dynamixel_sdk import *  # Uses Dynamixel SDK library


PORT = "/dev/ttyUSB0"
BAUD = 1000000
#DXL_ID = 1
ID_RANGE = range(1, 16)

portHandler = PortHandler(PORT)
packetHandler = PacketHandler(1.0)  # Protocol 1.0 for AX-12A

for DXL_ID in ID_RANGE:
 print(f"Port: {DXL_ID}")
 if portHandler.openPort():
    print("Port opened")
 else:
    print("Failed to open port")


 if portHandler.setBaudRate(BAUD):
    print("Baud set")
 else:
    print("Failed to set baud")


 dxl_model, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)


 if dxl_comm_result == COMM_SUCCESS:
    print(f"Ping success! Model number: {dxl_model}")
 else:
    print("Ping failed")

