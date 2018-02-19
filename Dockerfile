FROM ros:indigo-perception AS setup-data

WORKDIR /workspace/
COPY . .

RUN apt-get update && apt-get install -y \
  python-wstool python-rosdep ninja-build \
  libreadline6 libreadline6-dev

RUN mkdir -p carto_ws/src && \
  wstool init carto_ws/src env/carto.rosinstall

RUN carto_ws/src/cartographer/scripts/install_proto3.sh

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list; \
    rosdep init && \
    rosdep update; \
    rosdep install --from-paths carto_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y;

RUN /bin/bash -c 'cd carto_ws && \
  source /opt/ros/indigo/setup.bash && \
  catkin_make_isolated --install --use-ninja'

RUN /bin/bash -c 'cd catkin_ws && \
  source /opt/ros/indigo/setup.bash && \
  catkin_make --install'

FROM setup-data

WORKDIR /workspace/

CMD /bin/bash

