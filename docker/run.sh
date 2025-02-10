#!/bin/sh

docker run -it --rm \
    --name robocar \
    --ipc host \
    --network host \
    --device /dev/video0 \
    -v /dev/video0:/dev/video0 \
    --device /dev/dri \
    -v /dev/dri:/dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY:$XAUTHORITY \
    -e XAUTHORITY=$XAUTHORITY \
    -e DISPLAY=$DISPLAY \
    -v $(pwd):/host \
    ubix/robocar ${@}
