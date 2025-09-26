# XJTU-DXV-task-3-YaoBoWen
task-3:ceres+eigen
## 主要文件说明
- `main.cpp`：主程序，读取数据并拟合参数
- `include/trajectory_model.h`：弹道模型与残差定义
- `src/trajectory_model.cpp`：数据加载实现
i've forgotten the requirements of readme.md sorry .

code are below
//MAIN.CPP
#include <iostream>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include "trajectory_model.h"
#include <cmath>


double ComputeRMSE(const std::vector<TrajectoryObservation>& obs, double v0, double g, double k, double x0, double y0) {
    double sum2 = 0.0;
    int n = 0;
    for (const auto &o : obs) {
        double dt = o.t;
        double exp_term = std::exp(-k * dt);
        double x_pred = x0 + v0 / k * (1.0 - exp_term);
        double y_pred = y0 + (v0 / k + g / (k * k)) * (1.0 - exp_term) - g / k * dt;
        double dx = x_pred - o.x;
        double dy = y_pred - o.y;
        sum2 += dx*dx + dy*dy;
        n += 2;
    }
    return std::sqrt(sum2 / n);
}

struct WrapperResidual {
    double t,x_obs,y_obs,x0,y0;
    WrapperResidual(double t_, double x_, double y_, double x0_, double y0_) : t(t_), x_obs(x_), y_obs(y_), x0(x0_), y0(y0_) {}
    template<typename T>
    bool operator()(const T* const v0p, const T* const gp, const T* const log_kp, T* residuals) const {
        T k = ceres::exp(log_kp[0]);
        T dt = T(t);
        T exp_term = ceres::exp(-k * dt);
        T x_pred = T(x0) + v0p[0] / k * (T(1.0) - exp_term);
        T y_pred = T(y0) + (v0p[0] / k + gp[0] / (k * k)) * (T(1.0) - exp_term) - gp[0] / k * dt;
        residuals[0] = x_pred - T(x_obs);
        residuals[1] = y_pred - T(y_obs);
        return true;
    }
};

int main(int argc, char** argv) {
    
    std::string data_path;
    double true_v0 = NAN, true_g = NAN, true_k = NAN;

   
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) args.push_back(argv[i]);

    std::vector<std::string> positional;
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string &a = args[i];
        if (a.rfind("--", 0) == 0) {
          
            if (a == "--fix-g" || a == "--fix_g") {  }
            else if (a.rfind("--init-k",0) == 0) {  }
            else if (a.rfind("--min-k",0) == 0) { }
            else if (a.rfind("--max-k",0) == 0) { }
            else if (a.rfind("--fix-k",0) == 0) { }
            
            if (a.find('=') == std::string::npos && i+1 < args.size() && args[i+1].rfind("--",0) != 0) {
                ++i; 
            }
        } else {
            positional.push_back(a);
        }
    }
    if (!positional.empty()) data_path = positional[0];
    if (positional.size() >= 4) {
        true_v0 = std::stod(positional[1]);
        true_g  = std::stod(positional[2]);
        true_k  = std::stod(positional[3]);
    }

    std::vector<std::string> candidates;
    if (!data_path.empty()) candidates.push_back(data_path);
    candidates.push_back("data/trajectory.txt");
    candidates.push_back("../data/trajectory.txt");
    candidates.push_back("/home/ybw/chatgpt/data/trajectory.txt");

    std::vector<TrajectoryObservation> observations;
    std::string used_path;
    for (const auto &p : candidates) {
        observations = LoadTrajectoryData(p);
        if (!observations.empty()) { used_path = p; break; }
    }
    if (observations.empty()) {
        std::cerr << "No data loaded! Tried paths:" << std::endl;
        for (const auto &p : candidates) std::cerr << "  " << p << std::endl;
        return 1;
    }
    std::cout << "Loaded data from: " << used_path << " (" << observations.size() << " rows)" << std::endl;

    double x0 = observations[0].x;
    double y0 = observations[0].y;

    
    double v0 = 100.0;
    double g = 500.0;
    double log_k = std::log(0.1); 
    bool fix_g = false;

    double min_k = 0.01;
    double max_k = 5.0;
    bool has_min_k = false, has_max_k = false;
    bool fix_k = false;
    double fix_k_value = NAN;
  
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--fix-g" || a == "--fix_g") { fix_g = true; continue; }
        if (a.rfind("--init-k", 0) == 0) {
            std::string val;
            if (a.size() > 8 && a[8] == '=') val = a.substr(9);
            else if (i+1 < argc) { val = argv[++i]; }
            if (!val.empty()) log_k = std::log(std::stod(val));
            continue;
        }
        if (a.rfind("--min-k", 0) == 0) {
            std::string val;
            if (a.size() > 7 && a[7] == '=') val = a.substr(8);
            else if (i+1 < argc) { val = argv[++i]; }
            if (!val.empty()) { min_k = std::stod(val); has_min_k = true; }
            continue;
        }
        if (a.rfind("--max-k", 0) == 0) {
            std::string val;
            if (a.size() > 7 && a[7] == '=') val = a.substr(8);
            else if (i+1 < argc) { val = argv[++i]; }
            if (!val.empty()) { max_k = std::stod(val); has_max_k = true; }
            continue;
        }
        if (a.rfind("--fix-k", 0) == 0) {
            std::string val;
            if (a.size() > 6 && a[6] == '=') val = a.substr(7);
            else if (i+1 < argc) { val = argv[++i]; }
            if (!val.empty()) { fix_k = true; fix_k_value = std::stod(val); log_k = std::log(fix_k_value); }
            else { fix_k = true; }
            continue;
        }
      
    }

    ceres::Problem problem;
    for (const auto& obs : observations) {
        ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<WrapperResidual,2,1,1,1>(
            new WrapperResidual(obs.t, obs.x, obs.y, x0, y0));
        problem.AddResidualBlock(cost, nullptr, &v0, &g, &log_k);
    }

   
    problem.SetParameterLowerBound(&g, 0, 100.0);
    problem.SetParameterUpperBound(&g, 0, 1000.0);
    
    if (has_min_k) {
        double log_min_k = std::log(std::max(1e-12, min_k));
        problem.SetParameterLowerBound(&log_k, 0, log_min_k);
        std::cout << "Applied min_k = " << min_k << " (log_min_k=" << log_min_k << ")" << std::endl;
    }
    if (has_max_k) {
        double log_max_k = std::log(std::max(1e-12, max_k));
        problem.SetParameterUpperBound(&log_k, 0, log_max_k);
        std::cout << "Applied max_k = " << max_k << " (log_max_k=" << log_max_k << ")" << std::endl;
    }
    if (fix_k) {
        problem.SetParameterBlockConstant(&log_k);
        std::cout << "Fixing k to value: " << std::exp(log_k) << std::endl;
    }
   
    if (fix_g) {
        problem.SetParameterBlockConstant(&g);
        std::cout << "Fixing g to initial value: " << g << std::endl;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double k = std::exp(log_k);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Estimated v0: " << v0 << " px/s\nEstimated g: " << g << " px/s^2\nEstimated k: " << k << " 1/s" << std::endl;

    double rmse = ComputeRMSE(observations, v0, g, k, x0, y0);
    std::cout << "RMSE (per-component): " << rmse << std::endl;

    if (!std::isnan(true_v0)) {
        auto pct = [](double est, double truth){ return std::abs((est-truth)/truth)*100.0; };
        std::cout << "Percent errors: v0=" << pct(v0,true_v0) << "% g=" << pct(g,true_g) << "% k=" << pct(k,true_k) << "%" << std::endl;
    }

    return 0;
}

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
