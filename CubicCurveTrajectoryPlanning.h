#ifndef CUBIC_CURVE_TRAJECTORY_PLANNING_H
#define CUBIC_CURVE_TRAJECTORY_PLANNING_H

#include <functional>

class RobotDynamics {
public:
    // 构造函数：初始化惯性、摩擦系数和重力项
    RobotDynamics(double inertia, double friction, double gravity)
        : inertia_(inertia), friction_(friction), gravity_(gravity) {}

    // 计算力矩：基于惯性、摩擦和角加速度
    double calculateTorque(double angle, double angularVelocity, double angularAcceleration) const {
        return inertia_ * angularAcceleration + friction_ * angularVelocity + gravity_ * angle;
    }

private:
    double inertia_;   // 惯性
    double friction_;  // 摩擦系数
    double gravity_;   // 重力项
};

class CubicCurveTrajectoryPlanning {
public:
    // 构造函数：初始化三次曲线轨迹的参数
    CubicCurveTrajectoryPlanning(double theta1, double theta2, double theta_dot1,
                                 double theta_dot2, double totalTime, double timeStep);

    // 计算轨迹：遍历时间步长，输出角度和力矩到文件或回调
    void computeTrajectory(std::function<void(double)> getAngleCallback,
                           std::function<void(double)> getTorqueCallback,
                           const std::string& filename);

private:
    // 计算给定时间 t 的角度
    double getAngle(double t) const;

    // 计算给定时间 t 的角速度
    double getVelocity(double t) const;

    // 计算给定时间 t 的角加速度
    double getAcceleration(double t) const;

    // 计算给定时间 t 的力矩
    double getTorque(double t) const;

    // 计算三次曲线的系数
    void computeCoefficients();

    double theta1_, theta2_, theta_dot1_, theta_dot2_;  // 初始/末角度和初始/末速度
    double totalTime_, timeStep_;  // 总时间和时间步长
    double a0_, a1_, a2_, a3_;    // 三次曲线的系数
    RobotDynamics dynamics_;       // 动力学模型

};

#endif // CUBIC_CURVE_TRAJECTORY_PLANNING_H
