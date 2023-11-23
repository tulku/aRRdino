FROM osrf/ros:humble-desktop-full
RUN apt-get update && \ 
    apt-get install -y build-essential curl git libclang-dev python3-pip python3-vcstool tmux ros-humble-example-interfaces

RUN pip install git+https://github.com/colcon/colcon-cargo.git
RUN pip install git+https://github.com/colcon/colcon-ros-cargo.git

ARG USERNAME=eku
ARG PASSWORD=eku
ARG USERID=$UID
ARG HOME=/home/eku

RUN echo 'Create user' \
    && groupadd -f -g ${USERID} ${USERNAME} \
    && useradd -g ${USERID} -u ${USERID} -d ${HOME} -p $(python3 -c 'import crypt; print(crypt.crypt("${PASSWORD}", crypt.mksalt(crypt.METHOD_SHA512)))') -ms /bin/bash ${USERNAME} \
    && echo ${USERNAME}:${PASSWORD} | chpasswd \
    && usermod -aG sudo ${USERNAME} \
    && usermod -aG ${USERNAME} ${USERNAME} \
    && mkdir -p -m 0700 /run/user/${USERID} \
    && chown ${USERNAME}:${USERNAME} /run/user/${USERID}

RUN apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup
USER eku

RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
RUN echo 'source /home/eku/.cargo/env' >> /home/eku/.bashrc
RUN . /home/eku/.cargo/env && cargo install --debug cargo-ament-build
COPY setup.sh /setup.sh
