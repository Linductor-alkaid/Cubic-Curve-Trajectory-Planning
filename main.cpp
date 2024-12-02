#include <iostream>
#include "CubicCurveTrajectoryPlanning.h"
#include <cmath>

int main() {
    // 初始角度、末角度、初始速度、末速度、总时间、时间步长
    double theta1 = 0.0;    // 初始角度
    double theta2 = M_PI / 2; // 末角度（例如：90度）
    double theta_dot1 = 0.0; // 初始速度
    double theta_dot2 = 0.0; // 末速度
    double totalTime = 2.0;  // 总时间2秒
    double timeStep = 0.1;   // 时间步长0.1秒

    // 创建三次曲线轨迹规划对象
    CubicCurveTrajectoryPlanning trajectoryPlanner(theta1, theta2, theta_dot1, theta_dot2, totalTime, timeStep);

    // 设置输出文件名（例如：output.csv）
    std::string outputFile = "trajectory_output.csv";

    // 调用计算轨迹的函数，打印角度和力矩，并将结果保存到 CSV 文件
    trajectoryPlanner.computeTrajectory(
        [](double angle) { std::cout << "Current angle: " << angle << std::endl; },
        [](double torque) { std::cout << "Current torque: " << torque << std::endl; },
        outputFile // 将文件名传递给函数
    );

    std::cout << "Trajectory data saved to " << outputFile << std::endl;

    return 0;
}
