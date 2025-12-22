from dynamixel_sdk import *

port = PortHandler('/dev/ttyUSB0')
packet = PacketHandler(2.0)

if port.openPort():
    print("✅ Port opened")
else:
    print("❌ Failed to open port")

if port.setBaudRate(1000000):
    print("✅ Baud rate set to 1000000")
else:
    print("❌ Failed to set baud rate")

for dxl_id in [9, 10]:
    model_number, result, error = packet.ping(port, dxl_id)
    if result == COMM_SUCCESS:
        print(f"✅ Found motor ID {dxl_id}, model {model_number}")
    else:
        print(f"❌ No response from ID {dxl_id}")

