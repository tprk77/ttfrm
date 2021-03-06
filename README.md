# ttfrm #

[![CircleCI Status Shield][shield_circleci]][build_circleci]
[![Coveralls Status Shield][shield_coveralls]][build_coveralls]
[![Code Size Shield][shield_code_size]][ref_floppy_disk]
[![License Shield][shield_license]][file_license_md]

3D Rigid Transforms in C++ with frame checking. Ttfrm is built on top of
Eigen 3. It's implemented with quaternions instead of matrices, if you happen
to care about those sort of things.

Frame checking means that you can't accidentally mix up your transforms. Well
you can, but Ttfrm will helpfully crash your program. At least I think it's
helpful. Maybe it's best to see an example:

```cpp
const Quat rot = {0.7071, 0.0, -0.7071, 0.0};
const Vec3 trans = {1.0, 1.0, 1.0};

// Transform to X from World
const Tfrm<std::string> x_from_world("x", "world", rot, trans);

// Transform to Y from X
const Tfrm<std::string> y_from_x("y", "x", rot, trans);

// Transform to Z from Y
const Tfrm<std::string> z_from_y("z", "y", rot, trans);

// Compose transforms to get the transform to Z form World (?)
const Tfrm<std::string> z_from_world = z_from_y * x_from_world;

// ...Oops, forgot the transform to Y from X!
```

The above code will crash with the following exception:

```text
terminate called after throwing an instance of 'ttfrm::TfrmComposeException'
  what():  Cannot compose transforms ([z] <- [y]) and ([x] <- [world])
Aborted (core dumped)
```

The programmer can then go and fix the error!

```cpp
// We found the bug and everything is good this time!
const Tfrm<std::string> z_from_world = z_from_y * y_from_x * x_from_world;
```

## Building ##

The build uses Meson and Ninja. You will need to install those. On Ubuntu you
can probably run something like:

```text
$ sudo apt-get install python3-pip ninja-build
$ sudo pip3 install meson
```

You may optionally install dependencies for Eigen3, fmt, GTest, and SDL2.
(GTest and SDL2 are only used for tests and demos.) If you don't install them,
Meson should automatically download the dependencies for you.

```text
# This is optional, Meson can download dependencies
$ sudo apt-get install libeigen3-dev libfmt-dev libgtest-dev libsdl2-dev
```

Once the dependencies are in place, we can run the build:

```text
$ meson build
$ ninja -C build
```

## Testing ##

Run the tests using the following commands:

```text
$ meson build
$ ninja -C build
$ ./build/tfrm_test
$ ./build/tfrm_tree_test
```

## Benchmarking ##

Run the benchmarks using the following commands:

```text
$ meson build
$ ninja -C build
$ ./build/tfrm_bench
```

## Chaining Transforms ##

Transforms can be composed using a multiplication-like operator. So you can
write compositions like this:

```cpp
// Here's a good way to name your transforms!
const auto z_from_world = z_from_x * x_from_y * y_from_world;
```

Did you notice how the `x`'s matched up? How the `y`'s matched up? That's the
benefit of naming transforms with the `x_from_y` convention. We can easily
inspect the names of the transforms, and check that the transforms are composed
correctly. This doesn't happen when you name transforms with the `y_to_x`
convention, it's actually kind of a mess:

```cpp
// DON'T NAME TRANSFORMS THIS WAY, IT'S LAME!
const auto world_to_z = x_to_z * y_to_x * world_to_y;
```

See the difference? It's just a naming convention, but `x_from_y` is easier to
read. (At least I think so.)

## Poses As Transforms ##

The relationship between poses and transforms can be somewhat confusing. I
think that's because they're essentially two different names for the same
thing. A pose contains a rotation and translation relative to some frame,
exactly like a transform. If we want to transform a pose, we can do that by
treating the pose as a transform. To transform the pose, we just compose the
transform and the pose-like transform.

We typically use language like "Pose A in the World Frame." If we treat the
pose as a transform, we would instead say something like "Transform to the
World Frame from Frame A". Does that seem backwards? Consider that Pose A is at
the origin of Frame A. To get Pose A in the World Frame, we must transform the
origin of Frame A to the World Frame, which looks something like this:

```cpp
const auto pose_a_in_world = world_from_a * origin_in_a;
```

Of course, transforming the origin is completely unnecessary, and we can just
use an alias:

```cpp
const auto& pose_a_in_world = world_from_a;
```

If you only have the inverse, you could use that too:

```cpp
const auto pose_a_in_world = a_from_world.Inverse();
```

## Why Quaternions? ##

Quaternions have a couple things going for them. First, they use less space
than a rotation matrix. So you save a little memory with each transform.
Second, quaternions are faster for composing transforms. Third, quaternions are
faster for interpolating. Don't believe me? Run the benchmark. Here's an
example result:

```text
...
////////////////////
// CPU USAGE INFO //
////////////////////

Running ttfrm::Tfrm<int> benchmarks over 100M iterations... Done!
Running Eigen::Isometry3d benchmarks over 100M iterations... Done!

CPU usage summary:
  ttfrm::Tfrm<int> benchmarks:
    Apply:   Took 0.816 s (8 ns average)
    Compose: Took 2.681 s (26 ns average)
    Inverse: Took 3.271 s (32 ns average)
    Interp:  Took 2.359 s (23 ns average)
  Eigen::Isometry3d benchmarks:
    Apply:   Took 0.503 s (5 ns average)
    Compose: Took 3.433 s (34 ns average)
    Inverse: Took 1.602 s (16 ns average)
    Interp:  Took 4.692 s (469 ns average) [*]

[*]: 10M iterations only. Includes required quaternion conversions for slerp.
...
```

As you can see (and do try this at home!) composing transforms is faster with
quaternions. Applying a transform to a vector is slightly slower, but applying
a transform to another transform (such as a pose) is faster. So if your
workload involves chaining a bunch of transforms, or transforming a bunch of
poses, quaternions will give you better performance.

## Demo ##

In the `tests` directory there is a simple 2D Spirograph-like demo. It works by
composing several parameterized, circular transforms. You can run it using the
following commands:

```text
$ meson build
$ ninja -C build
$ ./build/spirograph_demo
```

<p align="center">
<img alt="Spirograph Demo" src="screenshots/spirograph.png"/>
</p>

<!-- Links -->

[build_circleci]: https://circleci.com/gh/tprk77/ttfrm
[build_coveralls]: https://coveralls.io/github/tprk77/ttfrm
[file_license_md]: https://github.com/tprk77/ttfrm/blob/master/LICENSE.md
[ref_floppy_disk]: https://en.wikipedia.org/wiki/History_of_the_floppy_disk
[shield_circleci]: https://img.shields.io/circleci/build/github/tprk77/ttfrm/master?token=9d161918287a4e250f49ae8df77fff90432ce358
[shield_code_size]: https://img.shields.io/github/languages/code-size/tprk77/ttfrm
[shield_coveralls]: https://img.shields.io/coveralls/github/tprk77/ttfrm
[shield_license]: https://img.shields.io/github/license/tprk77/ttfrm?color=informational

<!-- Local Variables: -->
<!-- fill-column: 79 -->
<!-- End: -->
