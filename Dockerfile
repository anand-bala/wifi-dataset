FROM ros:indigo-perception as build

WORKDIR /workspace/

RUN apt-get update && apt-get install -y \
  python-wstool python-rosdep ninja-build \
  realpath

COPY ./env ./env
COPY ./carto_ws ./carto_ws

RUN ./carto_ws/src/cartographer/scripts/install_proto3.sh; \
    rm /etc/ros/rosdep/sources.list.d/20-default.list; \
    rosdep init && \
    rosdep update; \
    apt-get update && \
    rosdep install --from-paths carto_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y;\
     /bin/bash -c 'cd carto_ws && \
    source /opt/ros/indigo/setup.bash && \
    catkin_make_isolated --install --use-ninja'


FROM build

WORKDIR /workspace/

COPY ./catkin_ws ./catkin_ws

RUN apt-get update && \
    rosdep install --from-paths catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y;

RUN /bin/bash -c 'cd catkin_ws && \
  source /opt/ros/indigo/setup.bash && \
  catkin_make install'

COPY ./scripts ./scripts
COPY ./env ./env 
VOLUME /data

ENTRYPOINT []
CMD [ "/bin/bash" ]


