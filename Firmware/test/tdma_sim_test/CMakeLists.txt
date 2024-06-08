cmake_minimum_required(VERSION 3.16.0)

project(tdma_sim_test)

target_compile_options(tdma_sim_test PUBLIC -Wall)
target_compile_options(tdma_sim_test PUBLIC -Wpedantic)
target_compile_features(tdma_sim_test PUBLIC cxx_std_17)
target_compile_options(tdma_sim_test PUBLIC -g)
target_compile_options(tdma_sim_test PUBLIC -O0)

add_executable(tdma_sim_test main.cpp)
