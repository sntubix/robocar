# Copyright (c) 2024 University of Luxembourg, MIT License

import sys, os, pickle
from threading import Lock
import numpy as np
import matplotlib.pyplot as plt
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from msg_interfaces.msg import Localization, MapWaypoint, Path, Objects2d
from cv_bridge import CvBridge


class Results(Node):
    def __init__(self, folder):
        super().__init__('results')

        self._filename = os.path.normpath(folder).split(os.path.sep)[-1]
        if (self._filename == ""):
            raise ValueError("invalid folder '" + folder + "'")
        self._filename = folder + "/" + self._filename
        if not os.path.exists(folder):
            os.mkdir(folder)

        self._v_t = np.array([], ndmin=1)
        self._v_speed = np.array([], ndmin=1)
        self._v_deviation = np.array([], ndmin=1)
        self._tfl_dists = np.array([], ndmin=1)

        self._mtx_w = Lock()
        self._waypoints = Path()
        self._objects = Objects2d()
        self._tfl_x = 0.0
        self._tfl_y = 0.0
        self._cv_bridge = CvBridge()
        self._init_frame_sec = -1
        self._frame_count = 0


    def dump(self):
        # subscribers
        self._sub_loc = self.create_subscription(Localization,
                                                 "localization/position",
                                                 self.on_loc, 1)
        self._sub_waypoints = self.create_subscription(Path,
                                                       "map/waypoints",
                                                       self.on_waypoints, 1)
        self._sub_objects = self.create_subscription(Objects2d,
                                                     "perception/camera/objects2d",
                                                     self.on_objects, 1)
        self._sub_image = self.create_subscription(Image,
                                                   "viz/perception/camera/objects2d",
                                                   self.on_frame, 1)

        # spin
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
        except KeyboardInterrupt:
            print("\ndumping data to '" + self._filename + "'")

        # check values
        if (np.size(self._v_t) == 0):
            print("no timestamp to dump")
            return
        if (np.size(self._v_speed) == 0):
            print("no speed data to dump")
            return
        if (np.size(self._v_deviation) == 0):
            print("no deviation data to dump")
            return

        # update time vectors
        for i in range(1, np.size(self._v_t)):
            self._v_t[i] -= self._v_t[0]
        self._v_t[0] = 0.0

        # dump data
        data = {
            "v_t": self._v_t,
            "v_speed": self._v_speed,
            "v_deviation": self._v_deviation,
            "tfl_dists": self._tfl_dists
        }
        pickle.dump(data, open(self._filename, "wb"))
        print("done")


    def plot(self):
        # load data
        data = pickle.load(open(self._filename, "rb"))
        self._v_t = data["v_t"]
        self._v_speed = data["v_speed"]
        self._v_deviation = data["v_deviation"]
        self._tfl_dists = data["tfl_dists"]

        # deviation smoothing
        window_size = 30
        kernel = np.ones(window_size) / window_size
        self._v_deviation = np.convolve(self._v_deviation, kernel, mode='same')

        FONT_SIZE = 35
        plt.rc('axes', labelsize=FONT_SIZE)
        plt.rc('axes', labelsize=FONT_SIZE)
        plt.rc('xtick', labelsize=FONT_SIZE)
        plt.rc('ytick', labelsize=FONT_SIZE)

        fig, ax_speed = plt.subplots()
        fig.set_size_inches(60.0, 10.0)
        ax_deviation = ax_speed.twinx()

        ax_speed.set_xlabel(r'$t$ [s]', fontsize=FONT_SIZE)
        ax_speed.set_ylabel(r'$speed$ [km/h]')
        ax_deviation.set_ylabel(r'$deviation$ [m]')

        ax_speed.set_xticks(np.arange(0.0, np.max(self._v_t), step=5.0))
        ax_speed.set_ylim(0, 40)
        ax_speed.set_xmargin(0.0)
        ax_deviation.set_ylim(-1.2, 1.2)

        LINE_WIDTH = 2.5
        speed_plot, = ax_speed.plot(self._v_t, self._v_speed, '-', linewidth=LINE_WIDTH, color='tab:blue', label='speed [km/h]')
        deviation_plot, = ax_deviation.plot(self._v_t, self._v_deviation, '-', linewidth=LINE_WIDTH, color='tab:green', label='deviation [m]')
        center_lane = np.zeros_like(self._v_t)
        center_lane_plot = ax_deviation.plot(self._v_t, center_lane, '--', linewidth=LINE_WIDTH, color='grey')

        plt.legend(handles=[speed_plot, deviation_plot], loc='upper right', fontsize=FONT_SIZE)
        plt.savefig(self._filename + ".svg")

        print(str(self._tfl_dists) + " " + str(np.mean(self._tfl_dists)))


    def compute_cte(self, loc: Localization, n_waypoint: MapWaypoint):
        # distance between direction line (path) and the vehicle
        cos_theta = np.cos(n_waypoint.yaw)
        sin_theta = np.sin(n_waypoint.yaw)
        x_0 = n_waypoint.x
        y_0 = n_waypoint.y
        x_1 = n_waypoint.x + cos_theta
        y_1 = n_waypoint.y + sin_theta
        a = y_0 - y_1
        b = x_1 - x_0
        c = x_0 * y_1 - x_1 * y_0
        return (a * loc.x + b * loc.y + c) / np.sqrt(a * a + b * b)


    def on_loc(self, loc: Localization):
        self._mtx_w.acquire()
        waypoints = self._waypoints
        objects = self._objects
        self._mtx_w.release()

        if np.size(waypoints.waypoints) == 0:
            return

        # search for nearest waypoint
        min_dist = float('inf')
        nearest_index = 0
        for i, waypoint in enumerate(waypoints.waypoints):
            dx = loc.x - waypoint.x
            dy = loc.y - waypoint.y
            dist = np.sqrt(dx * dx + dy * dy)

            if dist < min_dist:
                min_dist = dist
                nearest_index = i
        n_waypoint = waypoints.waypoints[nearest_index]

        self._v_t = np.append(self._v_t, [loc.header.stamp.sec + loc.header.stamp.nanosec / 1000000000.0], axis=0)
        self._v_speed = np.append(self._v_speed, [loc.vel * 3.6], axis=0)
        self._v_deviation = np.append(self._v_deviation, [self.compute_cte(loc, n_waypoint)], axis=0)

        # check traffic light detected
        tfl_detected = False
        for obj in objects.objects:
            if obj.type == 3:
                tfl_detected = True
                break
            if obj.type == 4:
                tfl_detected = True
                break
            if obj.type == 5:
                tfl_detected = True
                break
            if obj.type == 6:
                tfl_detected = True
                break

        # compute traffic light distance
        if (tfl_detected):
            prev_x = loc.x
            prev_y = loc.y
            tfl_dist = 0.0
            for i in range(nearest_index, np.size(waypoints.waypoints)):
                waypoint = waypoints.waypoints[i]

                dx = waypoint.x - prev_x
                dy = waypoint.y - prev_y
                tfl_dist += np.sqrt(dx * dx + dy * dy)
                prev_x = waypoint.x
                prev_y = waypoint.y

                if waypoint.tfl:
                    if (waypoint.x != self._tfl_x) and (waypoint.y != self._tfl_y):
                        self._tfl_x = waypoint.x
                        self._tfl_y = waypoint.y
                        self._tfl_dists = np.append(self._tfl_dists, [tfl_dist], axis=0)
                    break


    def on_waypoints(self, waypoints: Path):
        self._mtx_w.acquire()
        self._waypoints = waypoints
        self._mtx_w.release()


    def on_objects(self, objects: Objects2d):
        self._mtx_w.acquire()
        self._objects = objects
        self._mtx_w.release()


    def on_frame(self, frame: Image):
        if self._init_frame_sec == -1:
            self._init_frame_sec = frame.header.stamp.sec
        suffix = str(frame.header.stamp.sec - self._init_frame_sec) + "_" + str(self._frame_count)
        self._frame_count += 1

        cv_frame = self._cv_bridge.imgmsg_to_cv2(frame, desired_encoding='passthrough')
        cv2.imwrite(self._filename + "_frame_" + suffix + ".jpg", cv_frame)


def main(args=None):
    if (np.size(sys.argv) < 3):
        print("python results.py MODE FILENAME")
        exit()

    rclpy.init(args=args)
    results = Results(sys.argv[2])

    if (sys.argv[1] == "dump"):
        results.dump()

    if (sys.argv[1] == "plot"):
        results.plot()


if __name__ == '__main__':
    main()