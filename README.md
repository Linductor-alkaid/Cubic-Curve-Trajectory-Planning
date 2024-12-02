# Cubic Curve Trajectory Planning

## 项目简介

该项目实现了一个基于**三次曲线轨迹规划**（Cubic Spline）的机器人关节运动轨迹计算与力矩输出模型。通过给定机器人关节的初始角度、目标角度、初始速度、目标速度、总时间和时间步长，利用三次多项式曲线来生成平滑的运动轨迹，并计算每个时刻的力矩，提供机器人控制系统的必要输入。

本项目适用于机器人运动学控制系统，能够为机器人提供平滑的轨迹规划及关节力矩的前馈计算。

## 项目功能

- **三次曲线轨迹规划**：根据初始与目标角度、速度等参数，计算平滑的运动轨迹。
- **动力学建模**：考虑惯性、摩擦、重力等因素，计算每个时间步长的关节力矩。
- **回调机制**：通过回调函数输出每个时刻的角度和力矩，方便后续分析与调试。

## 安装与使用

### 环境要求

- C++11 或更高版本
- 任何标准C++编译器（如 GCC，Clang，MSVC）

### 安装步骤

1. 克隆或下载本项目的源代码。
   
   ```bash
   git clone https://github.com/your-repository/cubic-curve-trajectory-planning.git
   cd cubic-curve-trajectory-planning
   ```

2. 使用 CMake 构建项目（可选，取决于你的开发环境）。

   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

   或者，你也可以直接使用你的 IDE（如 VSCode, CLion）来编译运行项目。

### 使用方法

在项目的 `main.cpp` 文件中，我们展示了如何使用 `CubicCurveTrajectoryPlanning` 类来生成轨迹并输出角度与力矩。

1. 修改 `main.cpp` 中的参数，例如初始角度、目标角度、总时间等。
2. 使用回调函数输出轨迹和力矩数据，回调函数会在每个时间步执行。

以下是如何使用代码的一个示例：

```cpp
#include <iostream>
#include "CubicCurveTrajectoryPlanning.h"

int main() {
    // 初始化参数：初始角度、目标角度、初始速度、目标速度、总时间、时间步长
    double theta1 = 0.0;    // 初始角度
    double theta2 = M_PI / 2; // 目标角度（例如：90度）
    double theta_dot1 = 0.0; // 初始速度
    double theta_dot2 = 0.0; // 目标速度
    double totalTime = 2.0;  // 总时间2秒
    double timeStep = 0.1;   // 时间步长0.1秒

    // 创建轨迹规划对象
    CubicCurveTrajectoryPlanning trajectoryPlanner(theta1, theta2, theta_dot1, theta_dot2, totalTime, timeStep);

    // 计算轨迹并输出角度与力矩
    trajectoryPlanner.computeTrajectory(
        [](double angle) { std::cout << "Current angle: " << angle << std::endl; },
        [](double torque) { std::cout << "Current torque: " << torque << std::endl; }
    );

    return 0;
}
```

### 输出示例：

运行 `main.cpp` 后，你将会看到类似如下的输出，表示每个时间步的关节角度和力矩：

```text
Current angle: 0.0
Current torque: 0.0
Current angle: 0.005
Current torque: 0.1234
...
```

## 代码结构

### `CubicCurveTrajectoryPlanning.h`

- 定义了两个类：
  - `RobotDynamics`：用于计算力矩，基于关节角度、速度和加速度。
  - `CubicCurveTrajectoryPlanning`：实现了三次曲线轨迹规划，包含了角度、速度、加速度和力矩的计算。

### `CubicCurveTrajectoryPlanning.cpp`

- 实现了 `CubicCurveTrajectoryPlanning` 类的所有方法。
  - `computeCoefficients`：根据初始和目标条件计算三次曲线系数。
  - `computeTrajectory`：计算每个时刻的角度和力矩。
  - `getAngle`, `getVelocity`, `getAcceleration`, `getTorque`：分别计算角度、速度、加速度和力矩。

### `main.cpp`

- 主要的测试代码，展示如何创建 `CubicCurveTrajectoryPlanning` 对象，设置参数，并通过回调函数输出轨迹和力矩。

## 项目扩展

- **动力学模型扩展**：目前使用的是一个简单的惯性、摩擦、重力模型，未来可以考虑加入更多复杂的因素，如关节间的耦合、非线性摩擦、力矩饱和等。
- **轨迹规划方法扩展**：可以尝试使用其他轨迹规划算法，如五次曲线、Bezier曲线等，来应对不同的控制需求。
- **机器人控制**：将计算出的轨迹与实际的机器人硬件控制系统结合，完成闭环控制任务。
