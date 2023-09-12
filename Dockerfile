# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
# Added libcgal-dev package
RUN apt-get update && apt-get install -y \
  tzdata \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  xorg-dev \
  libgl1-mesa-dev \
  openssh-server \
  libcgal-dev \
  && rm -rf /var/lib/apt/lists/*

# Set timezone
RUN ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime && dpkg-reconfigure -f noninteractive tzdata

# SSHD 설정
RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config


WORKDIR /qslim

# Copy entire project directory
COPY . /qslim

# Download libigl
RUN git clone https://github.com/libigl/libigl.git /qslim/libigl

# Build your project
RUN rm -rf build && mkdir build && cd build && cmake .. && make

# SSH 포트 설정
EXPOSE 22

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=dialog

# SSHD 서비스 실행
CMD ["/usr/sbin/sshd", "-D"]