#!/usr/bin/python3

# Copyright (c) 2024 University of Luxembourg, MIT License

import cv2
from vidgear.gears import WriteGear

import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage


class TODStream(Node):
    def __init__(self):
        super().__init__('robocar_tod_stream')

        # qos_profile = rclpy.qos.QoSProfile(
        #     reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        #     history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        #     depth=1,
        #     durability=rclpy.qos.DurabilityPolicy.VOLATILE
        # )
        # self._pub_objects = self.create_publisher(CompressedImage, "sensors/camera/compressed", qos_profile)

    def run(self):
        output_params = {"-vcodec": "libx264", "-preset": "ultrafast", "-tune": "zerolatency",
                         "-metadata": "title=RoboCar", "-f": "rtsp", "-rtsp_transport": "tcp"}

        writer = WriteGear(
            output="rtsp://localhost:8554/robocar", logging=True, **output_params
        )

        cap = cv2.VideoCapture(0)
        # cap2 = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()
            # frame = cv2.flip(frame, -1)
            if frame is None:
                continue

            # ret2, frame2 = cap2.read()
            # if frame2 is None:
            #     continue

            # writer.write(cv2.hconcat([frame, frame2]))
            writer.write(frame)

        writer.close()


def main():
    rclpy.init()
    tod_stream = TODStream()
    tod_stream.run()
    #rclpy.spin(tod_stream)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
