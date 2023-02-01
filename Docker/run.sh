docker run --gpus all --rm -it  \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
    -v $HOME/FT_300s_ur3e:/root/FT_300s_ur3e \
    ur3e-tutorial bash
