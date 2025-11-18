cmake_minimum_required(VERSION 3.10)
project(DBoW2)

# create a cmake flag that can be passed here at build
option(LOCAL_ROOT_DIR "Set root dir to root" FALSE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set (dbow2_ROOR_DIR "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2")
if (${LOCAL_ROOT_DIR})
  set (dbow2_ROOR_DIR ./)
endif()

set(HDRS_DBOW2
  ${dbow2_ROOR_DIR}/DBoW2/BowVector.h
  ${dbow2_ROOR_DIR}/DBoW2/FORB.h 
  ${dbow2_ROOR_DIR}/DBoW2/FClass.h       
  ${dbow2_ROOR_DIR}/DBoW2/FeatureVector.h
  ${dbow2_ROOR_DIR}/DBoW2/ScoringObject.h   
  ${dbow2_ROOR_DIR}/DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  ${dbow2_ROOR_DIR}/DBoW2/BowVector.cpp
  ${dbow2_ROOR_DIR}/DBoW2/FORB.cpp      
  ${dbow2_ROOR_DIR}/DBoW2/FeatureVector.cpp
  ${dbow2_ROOR_DIR}/DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  ${dbow2_ROOR_DIR}/DUtils/Random.h
  ${dbow2_ROOR_DIR}/DUtils/Timestamp.h)
set(SRCS_DUTILS
  ${dbow2_ROOR_DIR}/DUtils/Random.cpp
  ${dbow2_ROOR_DIR}/DUtils/Timestamp.cpp)

# set(LIBRARY_OUTPUT_PATH ${dbow2_ROOR_DIR}/lib)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${dbow2_ROOR_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${dbow2_ROOR_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

