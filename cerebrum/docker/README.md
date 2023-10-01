# Roboost Docker Images

These images are used to execute ROS2 Roboost nodes on the Rock 5B.

## Build

```bash
docker build -t micro-ros-agent:latest -f docker/micro-ros-agent.Dockerfile .
```

## Run

```bash
docker run -it --rm --net=host -v /dev:/dev --name micro-ros-agent micro-ros-agent:latest
```

Alternatively, you can use the official micro-ros-agent docker image directly.

```bash
docker run -it --net=host microros/micro-ros-agent:humble udp4 -p 8888
```