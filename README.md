# RBY1-A

## Configuration
- RBY1
    - Model: A
    - Camera:
        - D405 x 2 (End-effector)
        - D435 (Head)

## Prerequisites

1. OpenCV
2. [Realsense](https://github.com/IntelRealSense/librealsense)


### Install Dependencies

```bash
# OpenCV
sudo apt install libopencv-dev

# Realsense
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update
sudo apt-get install librealsense2-utils librealsense2-dev
```