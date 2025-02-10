from vidgear.gears import WriteGear
import cv2

# define required FFmpeg parameters for your writer
output_params = {"-vcodec": "libx264", "-preset": "ultrafast", "-tune": "zerolatency", "-f": "rtsp", "-rtsp_transport": "tcp"}
#output_params = {"-vcodec": "libx264", "-preset": "ultrafast", "-tune": "zerolatency", "-f": "mpegts"}

writer = WriteGear(
    output="rtsp://localhost:8554/robocar", logging=True, **output_params
    #output="srt://localhost:8890?streamid=publish:mystream&pkt_size=1316", logging=True, **output_params
)

cap = cv2.VideoCapture(6)
cap2 = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, -1)
    if frame is None:
        continue

    ret2, frame2 = cap2.read()
    if frame2 is None:
        continue

    writer.write(cv2.hconcat([frame, frame2]))

# safely close writer
writer.close()
