# Makefile

BUILD_GRAPHICS := true
MESON_FLAGS := -Dgraphics=$(BUILD_GRAPHICS)

all: | build
	cd build && ninja

test: | build
	cd build && ninja
	./build/tfrm_test
	./build/tfrm_tree_test

bench: | build
	cd build && ninja
	./build/tfrm_bench

demo: | build
	cd build && ninja
	./build/spirograph_demo

install: | build
	cd build && ninja install

build:
	meson $(MESON_FLAGS) build

coverage: | build_cov
	cd build_cov && ninja tfrm_test tfrm_tree_test
	./build_cov/tfrm_test
	./build_cov/tfrm_tree_test

build_cov:
	CC=gcc CXX=g++ CXXFLAGS='--coverage -O0' meson -Dgraphics=false build_cov

clean:
	-rm -rf build build_cov
