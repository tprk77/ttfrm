# Makefile

BUILD_GRAPHICS := true
MESON_OPTIONS := -Dgraphics=$(BUILD_GRAPHICS)

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

# Used to configure the build dir
build:
	meson $(MESON_OPTIONS) build

# Used to reconfigure the build dir
reconfig:
	meson $(MESON_OPTIONS) build --reconfigure

clean:
	-rm -rf build
