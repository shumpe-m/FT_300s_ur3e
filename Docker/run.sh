docker run --gpus all --rm -it  \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
    -v $HOME/FT_300s_ur3e:/root/FT_300s_ur3e \
    FT_300s_ur3e bash
