cmake_minimum_required(VERSION 3.14)
project(pool_queue C)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "../lib")
set(CMAKE_C_STANDARD 99)

include_directories(inc)

add_library(pool_queue SHARED src/pool_queue.c inc/pool_queue.h)
