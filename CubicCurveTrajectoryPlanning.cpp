#include "CubicCurveTrajectoryPlanning.h"
#include <cmath>
#include <iostream>
#include <fstream>

// 构造函数，初始化曲线参数
CubicCurveTrajectoryPlanning::CubicCurveTrajectoryPlanning(double theta1, double theta2, double theta_dot1,
                                                           double theta_dot2, double totalTime, double timeStep)
    : theta1_(theta1), theta2_(theta2), theta_dot1_(theta_dot1), theta_dot2_(theta_dot2), totalTime_(totalTime),
      timeStep_(timeStep), dynamics_(1.0, 0.1, 0) {
    computeCoefficients();
}

// 计算三次曲线的系数
void CubicCurveTrajectoryPlanning::computeCoefficients() {
    double T = totalTime_;
    a0_ = theta1_;
    a1_ = theta_dot1_;
    a2_ = (3.0 / std::pow(T, 2)) * (theta2_ - theta1_) - (1.0 / T) * (theta_dot2_ + 2.0 * theta_dot1_);
    a3_ = (-2.0 / std::pow(T, 3)) * (theta2_ - theta1_) + (1.0 / std::pow(T, 2)) * (theta_dot2_ + theta_dot1_);
    std::cout << "a0: " << a0_ << ", a1: " << a1_ << ", a2: " << a2_ << ", a3: " << a3_ << std::endl;
}

// 计算给定时间 t 的角度
double CubicCurveTrajectoryPlanning::getAngle(double t) const {
    return a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3);
}

// 计算给定时间 t 的角速度
double CubicCurveTrajectoryPlanning::getVelocity(double t) const {
    return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t, 2);
}

// 计算给定时间 t 的角加速度
double CubicCurveTrajectoryPlanning::getAcceleration(double t) const {
    return 2 * a2_ + 6 * a3_ * t;
}

// 计算给定时间 t 的力矩
double CubicCurveTrajectoryPlanning::getTorque(double t) const {
    double theta_dot = getVelocity(t);
    double theta_ddot = getAcceleration(t);
    return dynamics_.calculateTorque(getAngle(t), theta_dot, theta_ddot);
}

// 轨迹计算：遍历时间步长，输出角度和力矩
void CubicCurveTrajectoryPlanning::computeTrajectory(std::function<void(double)> getAngleCallback,
                                                     std::function<void(double)> getTorqueCallback,
                                                     const std::string& filename) {
    std::ofstream outFile(filename);  // 打开输出文件
    if (!outFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    // 写入CSV文件的表头
    outFile << "Time,Angle,Torque\n";

    int numSteps = static_cast<int>(totalTime_ / timeStep_); // 计算步数
    for (int i = 0; i <= numSteps; ++i) {
        double t = i * timeStep_; // 当前时间
        double angle = getAngle(t);  // 当前角度
        double torque = getTorque(t); // 当前力矩

        // 写入当前时间的角度和力矩
        outFile << t << "," << angle << "," << torque << "\n";

        // 回调输出当前时间的角度和力矩
        getAngleCallback(angle);
        getTorqueCallback(torque);
    }

    outFile.close(); // 关闭文件
}

void CubicCurveTrajectoryPlanning::computeTrajectorynofile(std::function<void(double)> getAngleCallback,
                                                             std::function<void(double)> getTorqueCallback) {
    int numSteps = static_cast<int>(totalTime_ / timeStep_); // 计算步数
    for (int i = 0; i <= numSteps; ++i) {
        double t = i * timeStep_; // 当前时间
        double angle = getAngle(t);  // 当前角度
        double torque = getTorque(t); // 当前力矩

        // 回调输出当前时间的角度和力矩
        getAngleCallback(angle);
        getTorqueCallback(torque);
    }
}