# XJTU-DXV-task-3-YaoBoWen
task-3:ceres+eigen
## 主要文件说明
- `main.cpp`：主程序，读取数据并拟合参数
- `include/trajectory_model.h`：弹道模型与残差定义
- `src/trajectory_model.cpp`：数据加载实现
i've forgotten the requirements of readme.md sorry .



//CMAKELISTS
cmake_minimum_required(VERSION 3.10)
project(TrajectoryFitting)

set(CMAKE_CXX_STANDARD 14)

# Export compile commands for clang/clangd and editors
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} include)

# Some editors/tools expect the system Eigen include at /usr/include/eigen3
include_directories(/usr/include/eigen3)

add_executable(trajectory_fit main.cpp src/trajectory_model.cpp)

target_link_libraries(trajectory_fit ${CERES_LIBRARIES})
////TRAJECTORY_MODEL.CPP
#include "../include/trajectory_model.h"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<TrajectoryObservation> LoadTrajectoryData(const std::string& filename) {
    std::vector<TrajectoryObservation> data;
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return data;
    }
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double t, x, y;
        if (!(iss >> t >> x >> y)) continue;
        data.push_back({t, x, y});
    }
    return data;
}
//RESLUT
Estimated v0: 100.567 px/s
Estimated g: 160.026 px/s^2
Estimated k: 1.13232 1/s
RMSE (per-component): 0.0106115
