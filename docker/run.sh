#!/bin/sh

docker run -it --name robocar --rm \
    --ipc host \
    --device /dev/dri \
    -v /dev/dri:/dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY:$XAUTHORITY \
    -e XAUTHORITY=$XAUTHORITY \
    -e DISPLAY=$DISPLAY \
    -v $(pwd):/host \
    ubix/robocar ${@}
