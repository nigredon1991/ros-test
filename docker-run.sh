#!/bin/bash
sudo systemctl reload-or-restart docker

# without qt
# docker run --rm --name ros -v "$(pwd):/build" --workdir "/build" -it osrf/ros:jazzy-desktop
# For run qt app
docker run \
	--rm \
	--name ros \
	-v "$(pwd):/build" \
	-v "./src/ros_entrypoint.sh:/ros_entrypoint.sh" \
	--workdir "/build" \
	-e XDG_RUNTIME_DIR=/tmp \
	-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
	-e QT_QPA_PLATFORM=wayland \
	-e XDG_SESSION_TYPE=wayland \
	-v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
	-it ros-ngl bash
