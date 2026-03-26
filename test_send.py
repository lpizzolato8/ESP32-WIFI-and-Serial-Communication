import socket, time
from pose_frame import PoseFrame

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(20):
    frame = PoseFrame.pack(i, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)
    sock.sendto(frame, ("192.168.4.1", 4444))
    print(f"sent frame {i}")
    time.sleep(0.5)
sock.close()
