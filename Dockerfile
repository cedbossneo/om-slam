# Get a base image
FROM docker.io/ros:noetic-ros-base-focal as base
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
#   - git
#   - mosquitto broker
#   - nginx (remove default site to free up port 80)
RUN apt-get update && apt-get install --yes git python3-wstool python3-rosdep ninja-build stow

# Update rosdep
RUN rosdep update --rosdistro $ROS_DISTRO

# First stage: Pull the git and all submodules, other stages depend on it
FROM base as fetch

ENV DEBIAN_FRONTEND=noninteractive

COPY --link ./ /opt/om_slam
#RUN git clone https://github.com/ClemensElflein/om_slam /opt/om_slam

WORKDIR /opt/om_slam

RUN git submodule update --init --recursive

# Fetch the repo and assemble the list of dependencies. We will pull these in the next step and actually run install on them
# If the package list is the same as last time, the apt install step is cached as well which saves a lot of time.
# Since the list gets sorted, it will be the same each time and the cache will know that by file checksum in the COPY command.
# We can't use this stage as base for the next, because this stage is run every time and would therefore invalidate the cache of the next stage.
FROM fetch as dependencies

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /opt/om_slam

# This creates the sorted list of apt-get install commands.
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --simulate -r | \
    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install_list \
    && rm -rf /var/lib/apt/lists/*


# We can't derive this from "dependencies" because "dependencies" will be rebuilt every time, but apt install should only be done if needed
FROM base as assemble

ENV DEBIAN_FRONTEND=noninteractive

#Fetch the list of packages, this only changes if new dependencies have been added (only sometimes)
COPY --link --from=dependencies /apt-install_list /apt-install_list

RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) && \
    rm -rf /var/lib/apt/lists/*

# This will already have the submodules initialized, no need to clone again
COPY --link --from=fetch /opt/om_slam /opt/om_slam

RUN chmod +x /opt/om_slam/src/om_slam/src/publish_pose.py

WORKDIR /opt/om_slam

RUN src/lib/cartographer/scripts/install_abseil.sh

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /opt/om_slam/src && catkin_init_workspace && cd .. && catkin_make_isolated --use-ninja"

COPY .github/assets/slam_entrypoint.sh /slam_entrypoint.sh
RUN chmod +x /slam_entrypoint.sh

ENTRYPOINT ["/slam_entrypoint.sh"]
CMD ["bash", "-c", "roslaunch om_slam slam.launch --screen"]
