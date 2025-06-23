cmake_minimum_required (VERSION 3.16)
project(cart_pole_cpp)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)
find_library(MYMAT_LIB NAMES mymat PATHS "${CMAKE_CURRENT_SOURCE_DIR}/lib")
find_package(Eigen3 REQUIRED)


macro(macro1 arg)
  add_executable(cart_pole_cpp_${arg} cart_pole_cpp_${arg}.cpp)
  target_link_libraries (cart_pole_cpp_${arg} Eigen3::Eigen)
endmacro()

macro1(20200282)