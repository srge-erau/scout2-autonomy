FROM indro_base:latest

USER root

RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-pointcloud-to-laserscan && \
    rm -rf /var/lib/apt/lists/*

COPY cyclonedds.xml /home/cyclonedds.xml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///home/cyclonedds.xml

WORKDIR /home/scout_autonomy

CMD ["bash"]
