cmake_minimum_required(VERSION 3.10)
project(BoatSimulation)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(src)

add_executable(BoatSim 
    main.cpp 
    src/boat.cpp 
    src/constants.cpp
)

target_link_libraries(BoatSim GLEW::GLEW glfw OpenGL::GL)