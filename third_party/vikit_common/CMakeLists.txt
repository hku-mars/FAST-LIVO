SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Set build flags. Set IS_ARM on odroid board as environment variable
IF (WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /D_WINDOWS /D_REENTRANT")
ELSE()
    SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
    IF (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a") #-mfpu=neon 
    ELSE ()
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
    ENDIF ()
ENDIF ()

IF (CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE ()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF ()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# Set Sourcefiles
LIST(APPEND SOURCEFILES
#        src/atan_camera.cpp
#        src/omni_camera.cpp
#        src/math_utils.cpp
        src/vision.cpp
#        src/performance_monitor.cpp
        src/robust_cost.cpp
#        src/user_input_thread.cpp
        src/pinhole_camera.cpp
#        src/homography.cpp
#        src/img_align.cpp
        )

# Create vikit library
add_library(${PROJECT_NAME} STATIC ${SOURCEFILES})
if(WIN32)
set_target_properties(vikit_common
  PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Release"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Release"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Release"
)
endif()
# Include dirs
target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        )
target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${OpenCV_LIBS}
        Sophus
        )

set(${PROJECT_NAME} PARENT_SCOPE)
