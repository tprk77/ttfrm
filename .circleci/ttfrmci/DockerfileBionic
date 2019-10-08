# tprk77/ttfrmci

FROM circleci/buildpack-deps:bionic

# Install build tools (CMake is necessary to detect libfmt-dev)
RUN sudo apt-get install --no-install-recommends -qq -y \
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
        python3-yaml

# Install Meson
RUN sudo pip3 install meson==0.51

# Install code coverage tools
RUN sudo pip3 install cpp-coveralls

# Install dependencies
RUN sudo apt-get install --no-install-recommends -qq -y \
        libeigen3-dev \
        libfmt-dev \
        libgtest-dev
