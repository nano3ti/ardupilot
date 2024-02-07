docker run --rm -ti -e DISPLAY=$DISPLAY --privileged -v /tmp/.X11-unix:/tmp/.X11-unix  -v `pwd`:/ardupilot --name ardupilot ardupilot:latest bash
