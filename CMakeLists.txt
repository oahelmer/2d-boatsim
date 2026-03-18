cmake_minimum_required(VERSION 3.10)
project(BoatSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

# Manually find ONNX Runtime (since it's in your src directory)
set(ONNXRUNTIME_ROOT "${CMAKE_SOURCE_DIR}/src/onnxruntime-linux-x64-gpu-1.24.4")

# Check if ONNX Runtime exists
if(NOT EXISTS ${ONNXRUNTIME_ROOT})
    message(FATAL_ERROR "ONNX Runtime not found at ${ONNXRUNTIME_ROOT}")
endif()

# Set ONNX Runtime paths
set(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_ROOT}/include)
set(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_ROOT}/lib)

# Find the ONNX Runtime library
find_library(ONNXRUNTIME_LIB 
    NAMES onnxruntime onnxruntime.so
    PATHS ${ONNXRUNTIME_LIB_DIR}
    REQUIRED
)

# Include directories
include_directories(
    src
    ${ONNXRUNTIME_INCLUDE_DIR}
)

# Add executable
add_executable(BoatSim 
    main.cpp 
    src/boat.cpp 
    src/constants.cpp
    src/losguidance.cpp
    src/RLcontroller.cpp
)

# Link libraries
target_link_libraries(BoatSim 
    GLEW::GLEW 
    glfw 
    OpenGL::GL
    ${ONNXRUNTIME_LIB}
)

# Copy ONNX Runtime shared libraries to build directory on Linux
add_custom_command(TARGET BoatSim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${ONNXRUNTIME_LIB_DIR}
        $<TARGET_FILE_DIR:BoatSim>
    COMMENT "Copying ONNX Runtime libraries to output directory"
)

# Copy the ONNX model file to build directory
add_custom_command(TARGET BoatSim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/src/boat_policy.onnx
        $<TARGET_FILE_DIR:BoatSim>/boat_policy.onnx
    COMMENT "Copying ONNX model to output directory"
)