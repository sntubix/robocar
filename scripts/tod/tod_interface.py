#!/usr/bin/python3

# Copyright (c) 2024 University of Luxembourg, MIT License

import argparse
import time, json
from collections import deque
from typing import List, Tuple, Dict
import numpy as np
import pygame, cv2
import zmq
from ffpyplayer.player import MediaPlayer


class TodInterface():
    def __init__(self, w_width=1600, w_height=900, input="tx", v_host="localhost", cam_source="rtsp"):
        self._width = w_width
        self._cam_height = w_height
        self._height = round(1.14 * self._cam_height)
        self._input_conf = self._load_input_conf(input)
        self._cam_source = cam_source

        self._vehicle_host = v_host
        self._vehicle_data = {
            'ad_engaged': False,
            'steering': 0.0,
            'vel': 0.0
        }
        self._vehicle_cam = None

        # init latency
        self._prev_cam_stamp = 0
        self._cam_latencies = deque(maxlen=10)
        for i in range(self._cam_latencies.maxlen):
            self._cam_latencies.append(0.0)
        self._cam_latencies_sum = 0.0
        self._cam_latency = 0.0

    def _get_steering_ind(self, win_width: int, win_height: int, center: bool,
                          steering: float) -> List[Tuple]:
        offset_x = 0.5 * win_width
        if center:
            offset_y = 0.45 * win_height
        else:
            offset_y = 0.9 * win_height
        x1 = -9.0
        x2 = 9.0
        y1 = 50.0
        y2 = -50.0
        cos_theta = np.cos(steering)
        sin_theta = np.sin(steering)

        # (x1, y1)
        p1 = (x1 * cos_theta - y1 * sin_theta + offset_x,
              x1 * sin_theta + y1 * cos_theta + offset_y)
        # (x2, y1)
        p2 = (x2 * cos_theta - y1 * sin_theta + offset_x,
              x2 * sin_theta + y1 * cos_theta + offset_y)
        # (x2, y2)
        p3 = (x2 * cos_theta - y2 * sin_theta + offset_x,
              x2 * sin_theta + y2 * cos_theta + offset_y)
        # (x1, y2)
        p4 = (x1 * cos_theta - y2 * sin_theta + offset_x,
              x1 * sin_theta + y2 * cos_theta + offset_y)

        return [p1, p2, p3, p4]

    def _load_input_conf(self, input: str) -> Dict:
        input_conf = {
            'button_tod': 4,
            'button_ad': 8,
            'axis_steering': 0,
            'axis_throttle': 2,
            'axis_brake': 1,
            'cal_steering': 1.1,
            'cal_throttle': 2.7,
            'cal_brake': 2.5
        }

        if input == "tx":
            input_conf['button_tod'] = 4
            input_conf['button_ad'] = 8
            input_conf['axis_steering'] = 0
            input_conf['axis_throttle'] = 2
            input_conf['axis_brake'] = 1
            input_conf['cal_steering'] = 1.1
            input_conf['cal_throttle'] = 2.7
            input_conf['cal_brake'] = 2.5
            return input_conf

        raise ValueError(f"unsupported input: {input}")

    def run(self):
        _context = zmq.Context(2)
        _poller = zmq.Poller()

        _sock_data = _context.socket(zmq.PULL)
        _sock_data.connect(f"tcp://{self._vehicle_host}:5000")
        _poller.register(_sock_data, zmq.POLLIN)

        _sock_cmd = _context.socket(zmq.PUSH)
        _sock_cmd.connect(f"tcp://{self._vehicle_host}:5001")

        if self._cam_source == "rtsp":
            lib_opts = {'fflags':'nobuffer', 'flags':'low_delay'}
            _mp_cam = MediaPlayer(f"rtsp://{self._vehicle_host}:8554/robocar", lib_opts=lib_opts)

        # init pygame
        pygame.init()
        pygame.display.set_caption("RoboCar Teleoperated Driving")
        screen = pygame.display.set_mode((self._width, self._height), pygame.RESIZABLE)
        font = pygame.font.Font(None, 36)
        clock = pygame.time.Clock()

        # init pygame joystick
        use_joystick = False
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            use_joystick = True

        kb_steer_l = False
        kb_steer_r = False
        kb_throttle = False
        kb_brake = False
        prev_joy_tod = 0.0
        prev_joy_ad = 0.0
        tod_engaged = False
        ad_engaged = False
        ad_engage = False

        cmd = {
            'steering': 0.0,
            'throttle': 0.0,
            'brake': 0.0
        }

        running = True
        while running:
            kb_tod = False
            kb_ad = False
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_a:
                        kb_steer_l = True
                    if event.key == pygame.K_d:
                        kb_steer_r = True
                    if event.key == pygame.K_w:
                        kb_throttle = True
                    if event.key == pygame.K_SPACE:
                        kb_brake = True

                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_a:
                        kb_steer_l = False
                    if event.key == pygame.K_d:
                        kb_steer_r = False
                    if event.key == pygame.K_w:
                        kb_throttle = False
                    if event.key == pygame.K_SPACE:
                        kb_brake = False
                    if event.key == pygame.K_RALT:
                        kb_tod = True
                    if event.key == pygame.K_RSHIFT:
                        kb_ad = True

                if event.type == pygame.QUIT:
                    running = False

            if use_joystick:
                joy_tod = joystick.get_button(self._input_conf['button_tod'])
                if (joy_tod == 0.0) and (prev_joy_tod == 1.0):
                    tod_engaged = not tod_engaged
                prev_joy_tod = joy_tod
            else:
                if kb_tod:
                    tod_engaged = not tod_engaged

            if tod_engaged == True:
                steering = 0.0
                throttle = 0.0
                brake = 0.0

                if use_joystick:
                    joy_ad = joystick.get_button(self._input_conf['button_ad'])
                    if (joy_ad == 0.0) and (prev_joy_ad == 1.0):
                        ad_engage = not ad_engaged
                    prev_joy_ad = joy_ad

                    steering = (-joystick.get_axis(self._input_conf['axis_steering'])
                                * self._input_conf['cal_steering'])
                    if steering < -1.0:
                        steering = -1.0
                    if steering > 1.0:
                        steering = 1.0

                    throttle = (-joystick.get_axis(self._input_conf['axis_throttle']) + 1.0) / 2.0
                    if throttle < 0.0:
                        throttle = 0.0
                    if throttle > 1.0:
                        throttle = 1.0
                    throttle = float(np.pow(throttle, self._input_conf['cal_throttle']))

                    brake = (-joystick.get_axis(self._input_conf['axis_brake']) + 1.0) / 2.0
                    if brake < 0.0:
                        brake = 0.0
                    if brake > 1.0:
                        brake = 1.0
                    brake = float(np.pow(brake, self._input_conf['cal_brake']))

                else:
                    if kb_ad:
                        ad_engage = not ad_engaged
                    if kb_steer_l:
                        steering = 0.1
                    if kb_steer_r:
                        steering = -0.1
                    if kb_throttle:
                        throttle = 0.4
                    if kb_brake:
                        brake = 0.4

                cmd['steering'] = steering
                cmd['throttle'] = throttle
                cmd['brake'] = brake

                tod_msg = {
                    'stamp': round(time.time() * 1000.0),
                    'ad_engage': ad_engage,
                    'mode': 2,
                    'steering': steering,
                    'throttle': throttle,
                    'brake': brake
                }
                _sock_cmd.send_json(tod_msg)

            socks = dict(_poller.poll(timeout=20))

            if _sock_data in socks and socks[_sock_data] == zmq.POLLIN:
                msg_data = _sock_data.recv()
                self._vehicle_data = json.loads(str(msg_data, "utf-8"))
            ad_engaged = self._vehicle_data['ad_engaged']

            self._width, self._height = pygame.display.get_surface().get_size()
            self._cam_height = round(self._height / 1.14)
            v_cam = None

            if self._cam_source == "rtsp":
                #TODO fix this
                raise RuntimeError("RTSP support is incomplete")
                ff_frame, val = _mp_cam.get_frame()
                if (val != 'eof') and (ff_frame is not None):
                    ff_frame, _ = ff_frame
                    ff_frame_size = ff_frame.get_size()
                    #v_cam = cv2.Mat(np.asarray(ff_frame.to_memoryview()[0]).reshape(ff_frame_size[0], ff_frame_size[1], 3))
                    #print(len(ff_frame.to_memoryview()[0]))
                    #v_cam = cv2.imdecode(np.frombuffer(ff_frame.to_memoryview()[0], np.uint8).copy(), cv2.IMREAD_UNCHANGED)
                    #print(v_cam)

            if v_cam is not None:
                # compute frame latency
                now = time.time() * 1000.0
                cam_latency = now - self._prev_cam_stamp
                self._prev_cam_stamp = now

                # compute average latency
                self._cam_latencies_sum -= self._cam_latencies[0]
                self._cam_latencies_sum += cam_latency
                self._cam_latencies.append(cam_latency)
                cam_latency = round(self._cam_latencies_sum / self._cam_latencies.maxlen)
                self._cam_latency = cam_latency

                # process frame
                self._vehicle_cam = cv2.resize(v_cam, (self._width, self._cam_height))
                self._vehicle_cam = cv2.rotate(self._vehicle_cam, cv2.ROTATE_90_COUNTERCLOCKWISE)
                self._vehicle_cam = cv2.cvtColor(self._vehicle_cam, cv2.COLOR_BGR2RGB)

            # UI layout
            tod_w = 0.08
            tod_h = 0.93
            ads_w = 0.25
            ads_h = 0.93
            speed_w = 0.60
            speed_h = 0.93
            cmd_w = 0.75
            cmd_h = 0.93
            steer_ind_center = False

            # camera
            if self._vehicle_cam is not None:
                screen.blit(pygame.surfarray.make_surface(self._vehicle_cam), (0, 0))
            else:
                tod_h = 0.43
                ads_h = 0.43
                speed_h = 0.43
                cmd_h = 0.43
                steer_ind_center = True

            # TOD status
            if tod_engaged:
                screen.blit(font.render(f"TOD ON", True, (0, 255, 0)),
                            (tod_w * self._width, tod_h * self._height))
            else:
                screen.blit(font.render(f"TOD OFF", True, (255, 0, 0)),
                            (tod_w * self._width, tod_h * self._height))

            # ADS status
            if ad_engaged:
                screen.blit(font.render(f"ADS ON", True, (0, 255, 0)),
                            (ads_w * self._width, ads_h * self._height))
            else:
                screen.blit(font.render(f"ADS OFF", True, (255, 0, 0)),
                            (ads_w * self._width, ads_h * self._height))

            # steering indicator
            points = self._get_steering_ind(self._width, self._height,
                                            steer_ind_center, -self._vehicle_data['steering'])
            pygame.draw.polygon(screen, (0, 0, 255), points)

            # speed
            screen.blit(font.render(f"{round(self._vehicle_data['vel'] * 3.6)} km/h",
                        True, (255, 255, 255)), (speed_w * self._width, speed_h * self._height))

            # cmd
            screen.blit(font.render(f"cmd {cmd['steering']} {cmd['throttle']} {cmd['brake']}",
                        True, (255, 255, 255)), (cmd_w * self._width, cmd_h * self._height))

            pygame.display.update()
            screen.fill((0, 0, 0))
            clock.tick(120)


def main():
    parser = argparse.ArgumentParser(description="RoboCar Teleoperated Driving Interface")
    parser.add_argument("--w_width", type=int, default=1024, help="window width")
    parser.add_argument("--w_height", type=int, default=576, help="window height")
    parser.add_argument("--input", type=str, default="tx", help="input device")
    parser.add_argument("--v_host", type=str, default="localhost", help="vehicle hostname")
    parser.add_argument("--cam", type=str, default="rtsp", help="cam source: none, zmq or rtsp")
    args = parser.parse_args()

    if not (args.w_width > 0):
        raise ValueError("'w_width' must be > 0")
    if not (args.w_height > 0):
        raise ValueError("'w_height' must be > 0")

    tod_interface = TodInterface(args.w_width, args.w_height, args.input, args.v_host, args.cam)
    tod_interface.run()


if __name__ == '__main__':
    main()
