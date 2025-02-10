#!/bin/sh

ffplay -fflags nobuffer -flags low_delay -vf setpts=0 rtsp://localhost:8554/robocar
