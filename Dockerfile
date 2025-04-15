# Base image
FROM ros:humble

# Environment setup
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------------------------
# Build-time args for user/group so container user matches host
# ------------------------------------------------------------------------------
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=user

# ------------------------------------------------------------------------------
# Install OS & ROS dependencies
# ------------------------------------------------------------------------------
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
    sudo \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# Install Python packages
# ------------------------------------------------------------------------------
RUN pip3 install --upgrade pip
RUN pip3 install casadi pymap3d

# ------------------------------------------------------------------------------
# Build and install acados
# ------------------------------------------------------------------------------
WORKDIR /opt
RUN git clone https://github.com/acados/acados.git && \
    cd acados && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_PYTHON=ON && \
    make -j$(nproc) && \
    make install

# ------------------------------------------------------------------------------
# Install Rust & build t_renderer
# ------------------------------------------------------------------------------
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

RUN cd /opt/acados/bin && \
    git clone https://github.com/acados/tera_renderer.git && \
    cd tera_renderer && \
    cargo build --verbose --release && \
    cp target/release/t_renderer /opt/acados/bin/t_renderer

# ------------------------------------------------------------------------------
# Install Python bindings for acados
# ------------------------------------------------------------------------------
RUN pip3 install "setuptools<69" "setuptools_scm<8" wheel cython && \
    pip3 install --no-build-isolation -e /opt/acados/interfaces/acados_template

# ------------------------------------------------------------------------------
# Create a non-root user
# ------------------------------------------------------------------------------
RUN groupadd --gid ${GROUP_ID} ${USERNAME} && \
    useradd --uid ${USER_ID} --gid ${GROUP_ID} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ------------------------------------------------------------------------------
# Switch to user and copy in ROS 2 workspace
# ------------------------------------------------------------------------------
USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws
COPY --chown=${USERNAME}:${USERNAME} ./ros2_ws /home/${USERNAME}/ros2_ws

# ------------------------------------------------------------------------------
# Build ROS 2 workspace
# ------------------------------------------------------------------------------
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# ------------------------------------------------------------------------------
# Auto-source environment on container entry
# ------------------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export ACADOS_SOURCE_DIR=/opt/acados" >> /home/${USERNAME}/.bashrc && \
    echo "export LD_LIBRARY_PATH=/opt/acados/lib:\$LD_LIBRARY_PATH" >> /home/${USERNAME}/.bashrc && \
    echo "export PYTHONPATH=/opt/acados/interfaces/acados_template:\$PYTHONPATH" >> /home/${USERNAME}/.bashrc && \
    echo "export PATH=/opt/acados/bin:\$PATH" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
