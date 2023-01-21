docker run --gpus all --rm -it  \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
    -v $HOME/ur3e-tutorial:/root/ur3e-tutorial \
    ur3e-tutorial bash
