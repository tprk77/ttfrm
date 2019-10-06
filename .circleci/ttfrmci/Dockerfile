FROM ubuntu:18.04

# Install build tools (CMake is necessary to detect libfmt-dev)
RUN apt-get update -qq \
    && apt-get install --no-install-recommends -qq -y \
        clang \
        cmake \
        g++ \
        git \
        ninja-build \
        pkg-config \
        python3 \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        python3-yaml \
    && rm -rf /usr/share/doc/* /usr/share/man/*

# Install Meson
RUN pip3 install meson>=0.51

# Install code coverage tools
RUN pip3 install cpp-coveralls

# Install dependencies
RUN apt-get install --no-install-recommends -qq -y \
        libeigen3-dev \
        libfmt-dev \
        libgtest-dev
