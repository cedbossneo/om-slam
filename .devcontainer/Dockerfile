FROM docker.io/ros:noetic-ros-base-focal

ARG USERNAME=openmower
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && yes $USERNAME | passwd $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG dialout openmower

RUN apt-get update \
  && apt-get install -y ssh make curl unzip psutils libxext6 libxrender1 libxtst6 libxi6 libfreetype6 procps git \
    python3-pip ros-noetic-rqt-topic ros-noetic-rqt-tf-tree python3-wstool python3-rosdep ninja-build stow \
    rsync \
  && rm -rf /var/lib/apt/lists/*

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

ENV SHELL /bin/bash

RUN mkdir -p /home/ws/build /home/ws/install /home/ws/log \
  && chown -R $USERNAME:$USERNAME /home/ws

RUN ( \
    echo "source /opt/ros/noetic/setup.bash"; \
    echo "source /home/ws/devel_isolated/setup.bash"; \
    echo "export DISPLAY=:0"; \
  ) >> /home/$USERNAME/.bashrc

RUN ( \
    echo "PasswordAuthentication yes"; \
    echo "PermitRootLogin yes"; \
) >> /etc/ssh/sshd_config

RUN sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config
RUN mkdir /run/sshd

USER $USERNAME

CMD ["/usr/bin/sudo", "/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config"]
