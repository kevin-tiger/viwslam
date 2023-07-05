cmake_minimum_required(VERSION 3.1)
project(viwslam)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# # Eigen
# include(${CMAKE_SOURCE_DIR}/../cmake_modules/Eigen.cmake)
# # Ceres
# include(${CMAKE_SOURCE_DIR}/../cmake_modules/Ceres.cmake)
# # Pangolin
# include(${CMAKE_SOURCE_DIR}/../cmake_modules/Pangolin.cmake)
# # Opencv4
# include(${CMAKE_SOURCE_DIR}/../cmake_modules/OpenCV.cmake)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/src
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
)

add_library(${PROJECT_NAME} SHARED
src/utility/utility.cc
src/camera/PinholeCamera.cc
src/camera/CameraFactory.cc
src/viewer/viewer.cc
src/factor/integration_base.cc
src/factor/imu_encoder_factor.cc
src/factor/pose_local_parameterization.cc
src/factor/projectionTwoFrameOneCamFactor.cc
src/factor/marginalization_factor.cc
src/initial/solve_5pts.cc
src/initial/initial_sfm.cc
src/initial/initial_aligment.cc
src/initial/initial_ex_rotation.cc
src/featureTracker/feature_tracker.cc
src/estimator/parameters.cc
src/estimator/feature_manager.cc
src/estimator/estimator.cc
src/thirdparty/DBoW/BowVector.cpp
src/thirdparty/DBoW/FBrief.cpp
src/thirdparty/DBoW/FeatureVector.cpp
src/thirdparty/DBoW/QueryResults.cpp
src/thirdparty/DBoW/ScoringObject.cpp
src/thirdparty/DUtils/Random.cpp
src/thirdparty/DUtils/Timestamp.cpp
src/thirdparty/DVision/BRIEF.cpp
src/thirdparty/VocabularyBinary.cpp
src/loop_fusion/keyframe.cc
src/loop_fusion/pose_graph.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${CERES_LIBRARIES}
)

add_executable(run_viwslam app/run_viwslam.cc)
target_link_libraries(run_viwslam ${PROJECT_NAME})


