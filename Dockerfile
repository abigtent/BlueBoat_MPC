FROM ros:humble

# Environment setup
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Install OS & ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    libblas-dev \
    liblapack-dev \
    libtool \
    pkg-config \
    autoconf \
    gfortran \
    python3-dev \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*


# Install CasADi via pip
RUN pip3 install casadi

# Build ACADOS from source
WORKDIR /root
RUN git clone https://github.com/acados/acados.git && \
    cd acados && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_PYTHON=ON && \
    make -j$(nproc)

# Install acados Python interface in editable mode
# Install setuptools + setuptools_scm with compatible versions
RUN pip3 install --upgrade pip wheel cython && \
    pip3 install "setuptools<69" "setuptools_scm<8"

# Use --no-build-isolation to prevent setuptools_scm conflicts
RUN pip3 install --no-build-isolation -e /root/acados/interfaces/acados_template

# Set ACADOS environment variables
ENV ACADOS_SOURCE_DIR="/root/acados"
ENV LD_LIBRARY_PATH="/root/acados/lib"
ENV PYTHONPATH="/root/acados/interfaces/acados_template"

# Copy and build ROS 2 workspace
WORKDIR /root/ros2_ws
COPY ./ros2_ws ./ros2_ws

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Automatically source ROS + workspace on shell entry
#RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
 #   echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
