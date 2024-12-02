# 设置最低的 CMake 版本
cmake_minimum_required(VERSION 3.10)

# 项目的名称和版本
project(CubicCurveTrajectoryPlanning VERSION 1.0)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找所有源文件
file(GLOB SOURCES
    "CubicCurveTrajectoryPlanning.cpp"
    "CubicCurveTrajectoryPlanning.h"
    "main.cpp"
)

# 指定可执行文件的输出名称
add_executable(CubicCurveTrajectoryPlanning ${SOURCES})

# 如果你需要其他库，比如 math 库，可以在这里链接
# target_link_libraries(CubicCurveTrajectoryPlanning PRIVATE <library_name>)

# 设置构建输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
