cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(simulator VERSION 1.0)
project(tuning_test VERSION 1.0)
project(pid_controller_tests VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find boost
find_package(Boost 1.46.1 REQUIRED COMPONENTS filesystem system iostreams)

# check if boost was found
if(Boost_FOUND)
    message ("boost found")
else()
    message (FATAL_ERROR "Cannot find Boost")
endif()

# set sources
set(SOURCES_SIMULATOR
    src/main.cpp
)
set(SOURCES_TUNINGTEST
    test/tuning_test.cpp
)
set(SOURCES_PIDCONTROLLERTESTS
    test/pidControllerTests.cpp
)

# Add an executable with the above sources
add_executable(simulator ${SOURCES_SIMULATOR})
add_executable(tuning_test ${SOURCES_TUNINGTEST})
add_executable(pid_controller_tests ${SOURCES_PIDCONTROLLERTESTS})

# include directories
target_include_directories(simulator PUBLIC
                           "${PROJECT_SOURCE_DIR}/include"
                           )
target_include_directories(tuning_test PUBLIC
                           "${PROJECT_SOURCE_DIR}/include"
                           )
target_include_directories(pid_controller_tests PUBLIC
                           "${PROJECT_SOURCE_DIR}/include"
                           )

# linked libraries
target_link_libraries (simulator PRIVATE
                       Boost::filesystem
                       Boost::iostreams
                       )
target_link_libraries (tuning_test PRIVATE
                       Boost::filesystem
                       Boost::iostreams
                       )
target_link_libraries (pid_controller_tests PRIVATE
                       Boost::filesystem
                       Boost::iostreams
                       )
