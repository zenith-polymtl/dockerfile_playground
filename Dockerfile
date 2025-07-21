FROM osrf/ros:humble-desktop-full

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends \
        build-essential cmake python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-numpy \
      python3-numpy-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3 \
    python3-venv \
    python3-dev \
    python3-pip \
    libgpiod-dev \
    gpiod \
    python3-libgpiod \
    && python3 -m pip install --upgrade pip


RUN python3 -m pip install -U \
    zenmav \
    geopy \
    pymavlink


RUN python3 -m pip install -U \
    colcon-common-extensions \
    pymavlink \
    vcstool \
    rosdep \
    pytest-cov \
    pytest-repeat \
    pytest-rerunfailures \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    setuptools==65.7.0


RUN apt-get update && apt-get install -y \
  libxcb-cursor0 \
  libxcb-xinerama0 \
  libxcb-randr0 \
  libxcb-icccm4 \
  libxcb-keysyms1 \
  libxcb-shape0 \
  libxcb-render-util0 \
  libxcb-xkb1 \
  libxkbcommon-x11-0

RUN apt-get install -y \
  iputils-ping \
  iproute2 \
  apt-file

RUN printf '%s\n' \
    'Acquire::IndexTargets::deb::Contents-deb "false";' \
    'Acquire::IndexTargets::deb::Contents-udeb "false";' \
    'Acquire::Languages "none";' \
  > /etc/apt/apt.conf.d/99lean

RUN apt-get update && apt-get install -y python3-colcon-common-extensions \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-mavlink \
  geographiclib-tools

RUN apt-get update && apt-get install -y ros-humble-demo-nodes-cpp\
  python3-gpiozero

RUN pip3 install --no-cache-dir PyQt6

RUN pip3 install opencv-python

RUN python3 -m pip install --no-cache-dir \
        torch==2.3.0+cpu \
        torchvision==0.18.0+cpu \
        --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --no-cache-dir \
        "ultralytics>=8" 

RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    python3-rosdep \
    python3-argcomplete \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*


RUN python3 -m pip install -U \
    board \
    busio \
    libcamera \
    adafruit_pca9685 

WORKDIR /ros2_ws
COPY ros2_ws/src ./src

