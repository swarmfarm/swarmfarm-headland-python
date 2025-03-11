#!/bin/bash

# Build the Docker image
docker build -t lambda-env .

# Allow Docker containers to access the X11 server
xhost +local:docker

# Run the Docker container with the appropriate mounts and environment variable
docker run --rm -it \
  -v $(pwd):/app \
  -v $(pwd)/ci:/app/ci \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  lambda-env