cmake_minimum_required(VERSION 3.0)
project(TrajectoryVisualizer)

# Eigen 라이브러리 찾기
find_package(Eigen3 REQUIRED)

# Pangolin 라이브러리 찾기
find_package(Pangolin REQUIRED)

# OpenGL 라이브러리 찾기
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# 프로젝트에 포함할 소스 파일 지정
add_executable(trajectory_visualizer plotTrajectory.cpp)

# 필요한 헤더 파일과 라이브러리 링크
target_include_directories(trajectory_visualizer PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(trajectory_visualizer
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
)
