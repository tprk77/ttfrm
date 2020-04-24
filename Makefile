# Makefile

VERBOSE ?=
BUILD_GRAPHICS ?=
MESON_FLAGS ?= $(if $(BUILD_GRAPHICS),-Dgraphics=$(BUILD_GRAPHICS))
NINJA_FLAGS ?= $(if $(VERBOSE),-v)

all: | build
	cd build && ninja $(NINJA_FLAGS)

test: | build
	cd build && ninja $(NINJA_FLAGS)
	./build/tfrm_test
	./build/tfrm_tree_test

bench: | build
	cd build && ninja $(NINJA_FLAGS)
	./build/tfrm_bench

demo: | build
	cd build && ninja $(NINJA_FLAGS)
	./build/spirograph_demo

install: | build
	cd build && ninja $(NINJA_FLAGS) install

build:
	meson $(MESON_FLAGS) build

# We MUST use G++ in order to use the --coverage flag
MESON_FLAGS_CXX_COV := -Dcpp_args='--coverage -O0' -Dcpp_link_args='--coverage'
MESON_FLAGS_COV := -Dgraphics=false -D buildtype=plain $(MESON_FLAGS_CXX_COV)

coverage: | build_cov
	cd build_cov && ninja $(NINJA_FLAGS) tfrm_test tfrm_tree_test
	./build_cov/tfrm_test
	./build_cov/tfrm_tree_test

build_cov:
	CXX=g++ meson $(MESON_FLAGS_COV) build_cov

clean:
	-rm -rf build build_cov

BUILD_DEPENDS := python3-pip ninja-build

depends:
	sudo apt-get install $(BUILD_DEPENDS) # UBUNTU ONLY
	sudo -H pip3 install meson # UBUNTU ONLY

.PHONY: all test bench demo install coverage clean depends
