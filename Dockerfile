FROM osrf/ros:noetic-desktop-full-focal

# Install catkin
RUN sudo apt-get update -y
RUN sudo apt-get install -yq git wget python3-catkin-tools python3-osrf-pycommon

# Install zsh and OMzsh
RUN apt-get update -y
RUN apt-get install -yq zsh
RUN chsh -s $(which zsh)
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t tjkirch

# Create the whole project
RUN mkdir -p /track1/src/common
RUN mkdir -p /track1/src/perception/libs     
RUN git clone https://github.com/cyqian97/common_lib.git /track1/src/common/libs
RUN git clone https://github.com/cyqian97/roi_filters_lib.git /track1/src/perception/libs/roi_filters
RUN git clone https://github.com/cyqian97/object_builders_lib.git /track1/src/perception/libs/object_builders
RUN git clone https://github.com/cyqian97/segmenters_lib.git /track1/src/perception/libs/segmenters
RUN git clone https://github.com/cyqian97/feature_extractors_lib.git /track1/src/perception/libs/feature_extractors_lib
RUN git clone https://github.com/cyqian97/tracking_lib.git /track1/src/perception/libs/tracking_lib
WORKDIR "/track1"
RUN zsh -c "source /opt/ros/noetic/setup.zsh && catkin build -DCMAKE_BUILD_TYPE=Release"

# Copy a test rosbag
RUN mkdir -p /dataset/TAMU/TAMU-2022-02-17/
COPY 2022-02-17-18-12-35.bag /dataset/TAMU/TAMU-2022-02-17/

CMD ["zsh"]