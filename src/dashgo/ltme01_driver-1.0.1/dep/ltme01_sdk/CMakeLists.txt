cmake_minimum_required(VERSION 2.8)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  include("linux.cmake")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  include("win32.cmake")
endif()
