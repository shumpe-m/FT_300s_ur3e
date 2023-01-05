docker run --gpus all --rm -it  \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
    -v $HOME/ur3e-range-of-motion-survey:/root/ur3e-range-of-motion-survey \
    ur-range-survey bash
