FROM ros:indigo-perception

WORKDIR /workspace/
COPY . .

RUN /bin/bash ./install_cartographer.sh



