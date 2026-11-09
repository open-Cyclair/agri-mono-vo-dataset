# Environment of the GS-AgriField tools: MARV downloader and KITTI-like converter.
#
#   docker build -t gs-agrifield-tools .
#   docker run --rm gs-agrifield-tools --help
#
# Base image: ROS 2 Humble on Ubuntu 22.04, pinned by digest (x86-64 and ARM64).
FROM ros:humble@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-cv-bridge \
        ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/gs-agrifield
COPY requirements.txt requirements-converter.txt ./
RUN pip3 install --no-cache-dir -r requirements.txt -r requirements-converter.txt

COPY scripts/ scripts/
COPY msgs/ msgs/
COPY docker/entrypoint.sh /usr/local/bin/gs-agrifield
RUN chmod 755 /usr/local/bin/gs-agrifield

# The container runs as the host user (--user), who has no home directory.
ENV HOME=/tmp \
    ROS_LOG_DIR=/tmp/ros_log \
    PYTHONUNBUFFERED=1

WORKDIR /data
ENTRYPOINT ["gs-agrifield"]
CMD ["--help"]
