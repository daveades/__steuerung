
FROM osrf/ros:humble-desktop-full

ARG username=dev
ARG gid=1000
ARG uid=1000

WORKDIR /workspace

RUN groupadd -g ${gid} devgroup && useradd -g ${gid} -m -s /bin/bash -d /home/${username} -u ${uid} ${username}

USER ${username}