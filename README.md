# **BlueBoat_MPC**
This repository contains a real-time Nonlinear Model Predictive Control (NMPC) implementation using acados for the BlueBoat autonomous surface vessel. It leverages ROS 2 Humble, and is optimized for deployment on a NVIDIA Jetson Orin NX (16GB RAM). The NMPC controller supports both path following and obstacle avoidance, using nonlinear constraints for robust and reactive behavior.

The ROS node is built and run entirely inside a Docker container, managed through Docker Compose for easy deployment.

## Features:

✅ Real-time NMPC using acados

✅ Support for nonlinear constraints for safety and dynamic feasibility

✅ Path following of waypoints or reference trajectories

✅ Obstacle avoidance using external perception data (e.g. lidar or vision)

✅ Full Dockerized workflow for portability and reproducibility

✅ Tested on Jetson Orin NX (16GB RAM) with Ubuntu 22.04 and ROS 2 Humble

## 📦 Installation: 

### Prerequisites

Docker and Docker Compose installed on your Jetson Orin NX

### 🔧 Build the Docker Image

Clone the repository:
```bash
git clone https://github.com/abigtent/BlueBoat_MPC.git
cd Blueboat_MPC
```

Build the docker image with:
```bash
docker-compose build
```

### Running the image:
```bash
docker-compose up
```

Open a bash shell inside the running container:
```bash
docker exec -it nmpc_controller bash
```

