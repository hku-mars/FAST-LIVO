SET(PROJECT_NAME Sophus)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

IF (CMAKE_COMPILER_IS_GNUCXX )
   SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ")

  ADD_DEFINITIONS("-Wall -Werror -Wno-unused-variable
                   -Wno-unused-but-set-variable -Wno-unknown-pragmas ")
ENDIF()

################################################################################
# Add local path for finding packages, set the local version first
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
list( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )

#========================
# libs
#========================

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package( Eigen3 REQUIRED )

SET (SOURCE_DIR "sophus")
SET (CLASSES  so2
              se2
              se3
              so3
              scso3
              sim3
)

SET (SOURCES)

FOREACH(class ${CLASSES})
  LIST(APPEND SOURCES ${SOURCE_DIR}/${class}.cpp ${SOURCE_DIR}/${class}.h)
ENDFOREACH(class)

ADD_LIBRARY(${PROJECT_NAME} STATIC ${SOURCES})

#message(EIGEN3_INCLUDE_DIR:${EIGEN3_INCLUDE_DIR})
#message(EIGEN3_INCLUDE_DIR1:${PROJECT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        )

set(${PROJECT_NAME} PARENT_SCOPE)
