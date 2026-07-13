cmake_minimum_required(VERSION 3.16)
project(ctrl_core_tests C)
set(CMAKE_C_STANDARD 11)
include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.0)
FetchContent_MakeAvailable(unity)
enable_testing()

file(GLOB CTRL_SRCS  "${CMAKE_SOURCE_DIR}/../components/ctrl_core/*.c")
file(GLOB TEST_SRCS  "${CMAKE_SOURCE_DIR}/test_*.c")
include_directories("${CMAKE_SOURCE_DIR}/../components/ctrl_core/include")

foreach(tf ${TEST_SRCS})
    get_filename_component(name ${tf} NAME_WE)
    add_executable(${name} ${tf} ${CTRL_SRCS})
    target_link_libraries(${name} unity m)
    add_test(NAME ${name} COMMAND ${name})
endforeach()
