#!/usr/bin/python3

# Copyright (c) 2024 University of Luxembourg, MIT License

import sys
import numpy as np
from ultralytics import YOLO
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from msg_interfaces.msg import Object2d, Objects2d


class Detector(Node):
    def __init__(self, model_path):
        super().__init__('tfl_detector')
        self.conf = 0.25
        self.nms_iou = 0.7
        self.model = YOLO(model_path)

        self._pub_objects = self.create_publisher(Objects2d, "perception/camera/objects2d", 1)

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        self._sub_image = self.create_subscription(CompressedImage,
                                                   "sensors/camera/compressed",
                                                   self.on_frame, qos_profile)

    def on_frame(self, frame: CompressedImage):
        img = np.frombuffer(frame.data, np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)

        results = self.model.predict(img, verbose=False)
        for result in results:
            # img = result.plot()

            objects = []
            for i in range(result.boxes.xyxy.size(0)):
                objects.append(Object2d(header=frame.header,
                                        type=int(np.int32((result.boxes.cls[i] + 4).item())),
                                        conf=result.boxes.conf[i].item(),
                                        x1=result.boxes.xyxy[i][0].item(),
                                        y1=result.boxes.xyxy[i][1].item(),
                                        x2=result.boxes.xyxy[i][2].item(),
                                        y2=result.boxes.xyxy[i][3].item()))
                i += 1

            # img = cv2.resize(img, (1600, 900))
            # cv2.imshow('frame', img)
            # if cv2.waitKey(1) == ord('q'):
            #     return

        self._pub_objects.publish(Objects2d(objects=objects))


def main(args=None):
    rclpy.init(args=args)
    detector = Detector(sys.argv[1])
    rclpy.spin(detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()