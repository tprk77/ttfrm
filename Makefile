# Makefile

SHELL := bash
.SHELLFLAGS := -o errexit -o nounset -o pipefail -c
.DELETE_ON_ERROR:
MAKEFLAGS += --warn-undefined-variables
MAKEFLAGS += --no-builtin-rules
ifeq ($(origin .RECIPEPREFIX), undefined)
  $(error Please use a version of Make supporting .RECIPEPREFIX)
endif
.RECIPEPREFIX = >

# Important build variables
VERBOSE ?=
BUILD_GRAPHICS ?=
MESON_FLAGS ?= $(if $(BUILD_GRAPHICS),-Dgraphics=$(BUILD_GRAPHICS))
NINJA_FLAGS ?= $(if $(VERBOSE),-v)

# Automatically collect all sources
TTFRM_SRC_DIRS := ttfrm tests
TTFRM_SRCS := $(shell find $(TTFRM_SRC_DIRS) -type f -regex ".*\.[ch]pp$$")

all: build/build.sentinel

build:
> meson $(MESON_FLAGS) build

build/build.sentinel: $(TTFRM_SRCS) | build
> ninja $(NINJA_FLAGS) -C build
> touch build/build.sentinel

check: build/build.sentinel
> ./build/tfrm_test
> ./build/tfrm_tree_test

bench: build/build.sentinel
> ./build/tfrm_bench

demo: build/build.sentinel
> ./build/spirograph_demo

install: build/build.sentinel
> ninja $(NINJA_FLAGS) -C build install

# We MUST use G++ in order to use the --coverage flag
MESON_FLAGS_CXX_COV := -Dcpp_args='--coverage -O0' -Dcpp_link_args='--coverage'
MESON_FLAGS_COV := -Dgraphics=false -D buildtype=plain $(MESON_FLAGS_CXX_COV)

build_cov:
> CXX=g++ meson $(MESON_FLAGS_COV) build_cov

build_cov/build.sentinel: $(TTFRM_SRCS) | build_cov
> ninja $(NINJA_FLAGS) -C build_cov
> touch build_cov/build.sentinel

coverage: build_cov/build.sentinel
> ./build_cov/tfrm_test
> ./build_cov/tfrm_tree_test

clean:
> -rm -rf build build_cov

BUILD_DEPENDS := python3-pip ninja-build

depends:
> sudo apt-get install $(BUILD_DEPENDS) # UBUNTU ONLY
> sudo -H pip3 install meson # UBUNTU ONLY

.PHONY: all check bench demo install coverage clean depends
