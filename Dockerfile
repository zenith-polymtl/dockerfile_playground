FROM osrf/ros:humble-desktop-full

# Set noninteractive to avoid UI prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update & install ROS/colcon build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install core Python packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-dev \
    python3-pip \
    python3-numpy \
    python3-numpy-dev \
    libgpiod-dev \
    gpiod \
    python3-libgpiod \
    && python3 -m pip install --upgrade pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN python3 -m pip install -U \
    zenmav \
    geopy \
    pymavlink

RUN python3 -m pip install -U \
    colcon-common-extensions \
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

# Install ROS visualization & extra tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libxcb-cursor0 \
    libxcb-xinerama0 \
    libxcb-randr0 \
    libxcb-icccm4 \
    libxcb-keysyms1 \
    libxcb-shape0 \
    libxcb-render-util0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    iputils-ping \
    iproute2 \
    apt-file \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavlink \
    geographiclib-tools \
    ros-humble-demo-nodes-cpp \
    python3-gpiozero \
    ros-dev-tools \
    python3-rosdep \
    python3-argcomplete \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# Optimize apt
RUN printf '%s\n' \
    'Acquire::IndexTargets::deb::Contents-deb "false";' \
    'Acquire::IndexTargets::deb::Contents-udeb "false";' \
    'Acquire::Languages "none";' \
    > /etc/apt/apt.conf.d/99lean

# Enable 'universe' repo (for some Python camera libs)
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && add-apt-repository universe

# Install libcamera and bindings
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcamera-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Blinka + PCA9685
RUN python3 -m pip install --upgrade \
    adafruit-blinka \
    adafruit-circuitpython-pca9685

# Install opencv and ultralytics (YOLO, etc.)
RUN python3 -m pip cache purge && \
    pip3 install --no-cache-dir --resume-retries 3 opencv-python && \
    python3 -m pip install --no-cache-dir --resume-retries 10 \
        torch==2.3.0+cpu \
        torchvision==0.18.0+cpu \
        --index-url https://download.pytorch.org/whl/cpu && \
    python3 -m pip install --no-cache-dir "ultralytics>=8" && \
    python3 -m pip cache purge

# GUI dependencies for webview/X11 (not PyQt)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libx11-xcb1 \
    libglu1-mesa \
    libxrender1 \
    libxcomposite1 \
    libxcursor1 \
    libxdamage1 \
    libxrandr2 \
    libxtst6 \
    libxi6 \
    libglib2.0-0 \
    libsm6 \
    libnss3 \
    libatk1.0-0 \
    libatk-bridge2.0-0 \
    libgtk-3-0 \
    libxss1 \
    libasound2 \
    libpangocairo-1.0-0 \
    libgdk-pixbuf2.0-0 \
    libdbus-glib-1-2 \
    libnotify4 \
    libgconf-2-4 \
    x11-xserver-utils \
    webkit2gtk-4.0 \
    libwebkit2gtk-4.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install pywebview for GUI browser apps
RUN python3 -m pip install --no-cache-dir pywebview

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-gi \
    python3-gi-cairo \
    gir1.2-webkit2-4.0 \
    gir1.2-gtk-3.0 \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

WORKDIR /ros2_ws
COPY ros2_ws/src ./src
