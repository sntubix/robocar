#!/bin/sh

GPU_OPTION=""
if [ "$1" = "--gpu" ]; then
  GPU_OPTION="--gpus all"
fi

docker run -it --rm \
    --name robocar \
    --ipc host \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY:$XAUTHORITY \
    -e XAUTHORITY=$XAUTHORITY \
    -e DISPLAY=$DISPLAY \
    -v $(pwd):/ws \
    $GPU_OPTION \
    ubix/robocar
