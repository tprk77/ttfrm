# meson.build

project('ttfrm', 'cpp',
  version : '0.1.0',
  license : 'MIT',
  default_options : [
    'cpp_std=c++11',
    'buildtype=debugoptimized',
    'warning_level=3',
    'werror=true'
  ])

######################
# Declare Dependency #
######################

eigen_dep = dependency('eigen3', fallback : ['eigen3', 'eigen_dep'])
fmt_dep = dependency('fmt', fallback : ['fmt', 'fmt_dep'])
ttfrm_dep = declare_dependency(
  version : meson.project_version(),
  include_directories : include_directories('.'),
  dependencies : [eigen_dep, fmt_dep])

#########################
# Tests, Utilities, etc #
#########################

# Tests
gtest_dep = dependency('gtest', main : false, fallback : ['gtest', 'gtest_dep'])
tfrm_test = executable(
  'tfrm_test', 'tests/tfrm_test.cpp', dependencies : [ttfrm_dep, gtest_dep])
test('tfrm_test', tfrm_test)
tfrm_tree_test = executable(
  'tfrm_tree_test', 'tests/tfrm_tree_test.cpp', dependencies : [ttfrm_dep, gtest_dep])
test('tfrm_tree_test', tfrm_tree_test)

# Benchmark utility
tfrm_bench = executable('tfrm_bench', 'tests/tfrm_bench.cpp', dependencies : [ttfrm_dep])

# Spirograph-like demo
if get_option('graphics')
  sdl2_dep = dependency('sdl2', fallback : ['sdl2', 'sdl2_dep'])
  spirograph_demo = executable(
    'spirograph_demo', 'tests/spirograph_demo.cpp', dependencies : [ttfrm_dep, sdl2_dep])
endif

###########
# Install #
###########

headers = [
  'ttfrm/tfrm.hpp',
  'ttfrm/tfrm_tree.hpp'
]

install_headers(headers, subdir : 'ttfrm')

pkg_mod = import('pkgconfig')
pkg_mod.generate(
  name : 'ttfrm',
  filebase : 'ttfrm',
  description : '3D Rigid Transforms in C++',
  subdirs : 'ttfrm',
  version : meson.project_version())
