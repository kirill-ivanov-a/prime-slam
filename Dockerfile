FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

ARG OPENCV_VERSION=4.6.0

ARG CMAKE_VERSION=3.20.0

# Install essential build tools and dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    ffmpeg \
    libatlas-base-dev \
    libavcodec-dev \
    libavformat-dev \
    libavdevice-dev \
    libfontconfig1 \
    libgtk-3-dev \
    libjpeg-dev \
    libpng-dev \
    libsm6 \
    libswscale-dev \
    libtiff-dev \
    libv4l-dev \
    libx264-dev \
    libxext6 \
    libxrender1 \
    libxvidcore-dev \
    locales \
    python3-pip \
    unzip \
    wget \
    yasm \
    cmake \
    git \
    libeigen3-dev \
    libboost-all-dev

# install opencv
RUN wget -O opencv-${OPENCV_VERSION}.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
    && wget -O opencv_contrib-${OPENCV_VERSION}.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip \
    && unzip opencv-${OPENCV_VERSION}.zip \
    && unzip opencv_contrib-${OPENCV_VERSION}.zip \
    && mkdir opencv-${OPENCV_VERSION}/build  \
    && cd opencv-${OPENCV_VERSION}/build \
    && cmake  \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D WITH_OPENCL=OFF \
    -D WITH_OPENJPEG=OFF \
    -D WITH_JASPER=OFF \
    -D WITH_OPENEXR=OFF \
    -D WITH_V4L=OFF \
    -D WITH_FFMPEG=OFF \
    -D WITH_GSTREAMER=OFF \
    -D BUILD_JAVA=OFF \
    -D WITH_CUDA=OFF \
    -D BUILD_TESTS=OFF \
    -D WITH_CUDNN=OFF \
    -D OPENCV_DNN_CUDA=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib-${OPENCV_VERSION}/modules \
    -D HAVE_opencv_python3=ON \
    -D BUILD_EXAMPLES=OFF .. \
    && make install -j4 \
    && ldconfig \
    && rm -rf opencv-${OPENCV_VERSION}.zip opencv_contrib-${OPENCV_VERSION}.zip \
    opencv-${OPENCV_VERSION} opencv_contrib-${OPENCV_VERSION}

RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh && \
    sh cmake-${CMAKE_VERSION}-Linux-x86_64.sh --skip-license --prefix=/usr/local

# Create a directory for the source code and navigate to it
WORKDIR /open3d

# Clone the Open3D repository
RUN git clone --recursive https://github.com/intel-isl/Open3D.git .

# Build and install Open3D
RUN mkdir build && cd build && cmake -DBUILD_GUI=OFF .. && make -j4 && make install

WORKDIR /app

COPY . /app

RUN mkdir build && \
    cd build && \
    cmake .. && \
    make

ENTRYPOINT ["./build/my_cpp_project/examples/main"]
