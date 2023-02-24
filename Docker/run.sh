docker run --name ft_300s_ur3e --gpus all --rm -it  \
    -u $(id -u $USER):$(id -g $USER) \
    -v /etc/group:/etc/group:ro \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/shadow:/etc/shadow:ro \
    -v /etc/sudoers.d:/etc/sudoers.d:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
    -v $HOME/FT_300s_ur3e:/root/FT_300s_ur3e \
    ft_300s_ur3e bash
