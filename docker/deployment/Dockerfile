ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
  ros-dev-tools \
  wget \
  curl \
  python3-vcstool 

#Install Node.js and npm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
RUN . /root/.nvm/nvm.sh && nvm install 20

#Install the camco packages
WORKDIR /root/camco_ws
RUN mkdir ./src
RUN git clone https://github.com/YazanRihan/Campus-Courier.git --branch main ./src/Campus-Courier
RUN vcs import src < ./src/Campus-Courier/dependencies_${ROS_DISTRO}.repos
RUN apt update && apt upgrade -y \
    && rosdep update \
    && rosdep install -y --ignore-src --from-paths src -r
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install

#Call the entrypoint script
RUN chmod +x ./src/Campus-Courier/scripts/entrypoint.sh
ENTRYPOINT ["./src/Campus-Courier/scripts/entrypoint.sh"]
#By default, run entrypoint.sh with arg1 set to nav-server (Run the container for server only) and arg2 to w12.yaml (map of W12)
CMD ["nav-server", 'w12.yaml']