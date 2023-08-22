# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && apt-get install -y \
  tzdata \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  xorg-dev \
  libgl1-mesa-dev \
  && rm -rf /var/lib/apt/lists/*

# Set timezone
RUN ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime && dpkg-reconfigure -f noninteractive tzdata

WORKDIR /qslim

# Copy entire project directory
COPY . /qslim

# Download libigl
RUN git clone https://github.com/libigl/libigl.git /qslim/libigl

# Build your project
RUN rm -rf build && mkdir build && cd build && cmake .. && make

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=dialog
