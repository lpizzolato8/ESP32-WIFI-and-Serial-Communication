import socket
import time
from pose_frame import PoseFrame

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(20):
    frame = PoseFrame.pack(pos_x=1.0, pos_y=2.0, pos_z=3.0,
                           quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)
    sock.sendto(frame, ("192.168.4.1", 4444))
    print(f"sent frame {i}")
    time.sleep(0.5)
sock.close()
