cmake_minimum_required(VERSION 3.28.1)
project(EmbeddedMachineLearn C)

set(CMAKE_C_STANDARD 11)

file(GLOB_RECURSE SOURCES
        Src/*.*
)

include_directories(
        Inc
)
add_compile_options(-Dgenann_act=genann_act_sigmoid_cached)

find_package(GSL)

add_executable(EmbeddedMachineLearn main.c ${SOURCES})

target_link_libraries(EmbeddedMachineLearn PRIVATE GSL::gsl)