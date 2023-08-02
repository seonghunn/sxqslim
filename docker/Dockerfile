FROM ubuntu:20.04

# Set timezone
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install required packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    xorg-dev \
    libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /qem

COPY ../ /qem

# Download libigl
RUN git clone https://github.com/libigl/libigl.git /qem/libigl

# Build your project
RUN rm -rf build && mkdir build && cd build && cmake .. && make
