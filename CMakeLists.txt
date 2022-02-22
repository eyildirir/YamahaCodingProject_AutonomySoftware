cmake_minimum_required(VERSION 3.10)

######### SIMULATOR #########

# set the project name and version
project(simulator VERSION 1.0)

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


set(SOURCES
    src/main.cpp
)

# Add an executable with the above sources
add_executable(simulator ${SOURCES})

target_include_directories(simulator PUBLIC
                           "${PROJECT_SOURCE_DIR}/include"
                           )

target_link_libraries (simulator PRIVATE
                       Boost::filesystem
                       Boost::iostreams
                       )




######### TUNING TEST #########

# set the project name and version
project(tuning_test VERSION 1.0)

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


set(SOURCES
    test/tuning_test.cpp
)

# Add an executable with the above sources
add_executable(tuning_test ${SOURCES})

target_include_directories(tuning_test PUBLIC
                           "${PROJECT_SOURCE_DIR}/include"
                           )

target_link_libraries (tuning_test PRIVATE
                       Boost::filesystem
                       Boost::iostreams
                       )



######### PID CONTROLLER TESTS #########

# set the project name and version
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


set(SOURCES
    test/pidControllerTests.cpp
)

# Add an executable with the above sources
add_executable(pid_controller_tests ${SOURCES})

target_include_directories(pid_controller_tests PUBLIC
                           "${PROJECT_SOURCE_DIR}/include"
                           )

target_link_libraries (pid_controller_tests PRIVATE
                       Boost::filesystem
                       Boost::iostreams
                       )