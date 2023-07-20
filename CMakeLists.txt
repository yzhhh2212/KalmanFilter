cmake_minimum_required(VERSION 3.10)
project(IMU_Kalman_Filter)

set(CMAKE_CXX_STANDARD 14)

# Add the path to the FindEigen3.cmake file. This may not be necessary if Eigen3 is installed system-wide.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Find the Eigen3 library
find_package(Eigen3 REQUIRED)

# Add the include directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/matplotlib-cpp)

# Add the source files
set(SOURCE_FILES
    src/main.cpp
    src/kalmanfilter.cpp
    src/SimulateImuData.cpp
)

# Add the executable
add_executable(IMU_Kalman_Filter ${SOURCE_FILES})

# Link the Eigen3 library
target_link_libraries(IMU_Kalman_Filter ${EIGEN3_LIBS})

# Link the Python libraries for matplotlib-cpp
find_package(PythonLibs 3)
target_include_directories(IMU_Kalman_Filter PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(IMU_Kalman_Filter ${PYTHON_LIBRARIES})
