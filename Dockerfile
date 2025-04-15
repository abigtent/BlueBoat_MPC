# Base image
FROM ros:humble

# Environment setup
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------------------------
# 1) Create build-time args for user/group, so the container user matches your host
# ------------------------------------------------------------------------------
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=user

# Install OS & ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    curl \
    git \
    wget \
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

# ------------------------------------------------------------------------------
# 2) Create a non-root user with sudo privileges
RUN groupadd --gid ${GROUP_ID} ${USERNAME} && \
useradd --uid ${USER_ID} --gid ${GROUP_ID} -m -s /bin/bash ${USERNAME} && \
apt-get update && apt-get install -y sudo && \
echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
apt-get clean && rm -rf /var/lib/apt/lists/*

# Switch to the new user
USER ${USERNAME}

# Install CasADi
RUN pip3 install casadi

# Install pymap3d
RUN pip install pymap3d

# Build and install acados
WORKDIR /root
RUN git clone https://github.com/acados/acados.git && \
    cd acados && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_PYTHON=ON && \
    make -j$(nproc) && \
    make install

# Install Rust using rustup (this installs both rustc and cargo)
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Clone tera_renderer, build it, then copy the binary
WORKDIR /root/acados/bin/
RUN git clone https://github.com/acados/tera_renderer.git && \
    cd tera_renderer && \
    cargo build --verbose --release && \
    cp target/release/t_renderer /root/acados/bin/t_renderer

# Install Python bindings for acados
RUN pip3 install --upgrade pip wheel cython && \
    pip3 install "setuptools<69" "setuptools_scm<8" && \
    pip3 install --no-build-isolation -e /root/acados/interfaces/acados_template

# Set acados environment variables
ENV ACADOS_SOURCE_DIR="/root/acados"
ENV LD_LIBRARY_PATH="/root/acados/lib:$LD_LIBRARY_PATH"
ENV PYTHONPATH="/root/acados/interfaces/acados_template:$PYTHONPATH"
ENV PATH="/root/acados/bin:$PATH"

# Copy and build ROS 2 workspace
WORKDIR /root/ros2_ws
COPY ./ros2_ws/src /root/ros2_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Auto-source ROS & workspace on container entry
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ACADOS_SOURCE_DIR=/root/acados" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/root/acados/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "export PYTHONPATH=/root/acados/interfaces/acados_template:\$PYTHONPATH" >> ~/.bashrc && \
    echo "export PATH=/root/acados/bin:\$PATH" >> ~/.bashrc



CMD ["/bin/bash"]
