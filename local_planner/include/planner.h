#pragma once
#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"
#include <limits>
#include <vector>

#include "egoVehicle.h"
#include "scenario.h"

#define INF 60000

using namespace std;
// 在这个框架中, 以后所有的局部轨迹规划算法都在planner.h文件中实现
// 输入:
// 1.全局的道路信息(参考线就是单独的车道线信息)x, y, theta, f_vec,road_index
// 2.当前的场景信息(scenario中的动态障碍物)x[], y[], theta[], Index, velocity
// 3.当前的主车状态 LocalTrajectory_[0]: x, y, vx, vy, ax, ay, theta, road_index
// 输出:
// 局部轨迹: x, y, vx, vy, ax, ay, theta, road_index

// 这是第一个planner, 将nmpc问题局部线性化为QP问题进行迭代求解
class Planner_MPC
{
public:
  Planner_MPC() = default;
  ~Planner_MPC() = default;
  void planner(Scenario &scenario, EgoVehicle &ego);
  // 辅助函数：初始化暖启动
  void Initialization(EgoVehicle &ego);

private:
  // step1: 整理输入的数据，得到当前时刻的所有输入信息
  // void GetNowInformation(Scenario &scenario, EgoVehicle &ego);
  // step2: 构建目标函数的Q矩阵
  Eigen::SparseMatrix<double> CostFunction(Scenario &scenario, EgoVehicle &ego);
  // step3: 构建不等式约束的A矩阵以及约束边界
  void ConstriantFunction(Scenario &scenario, EgoVehicle &ego);
  // step4: 求解问题
  bool Solution(EgoVehicle &ego);
  // step5: 从优化结果放入LocalTrajectory_
  void ResultToTrajectory(Eigen::VectorXd &solution, EgoVehicle &ego);

public:
  int predict_step_ = 50;                             // mpc预测长度, 设置为50步
  int step_scale_ = 6;                                // 每一步的规模：x, y, vx, vy, ax, ay， 6个
  int input_scale_ = predict_step_ * step_scale_ + 1; // 优化变量的总规模
  int eq_constraint_number_ =
      predict_step_ * 4; // 等式约束的规模: 运动学模型约束(4 *predict_step_)

  int ineq_constraint_number_ = (2 + 2 + 2) * predict_step_ +
                                2; // 不等式约束的规模:速度约束 + 加速度约束 +
                                   // 边界约束(包含了障碍物约束) + 终端速度为0

  double weight_refline_ = 20; // 参考线惩罚权重
  double weight_R_ =
      10;           // 控制量变化惩罚权重 设为10的整体效果也不错，在减速后也会想办法超车
  double dt_ = 0.1; // 两点间的时间间隔为0.1s

  vector<double> warm_start_; // 上一帧规划结果[x0, y0, vx0, vy0, ax0, ay0,...]
  vector<tuple<double, double, double, double, double, double, double, int>>
      LastLocalTrajectory_;
  vector<Eigen::SparseMatrix<double>> Q; // 目标矩阵
  vector<Eigen::VectorXd> Derivative;
  vector<Eigen::VectorXd> bound;
  vector<Eigen::VectorXd> gradient;
  vector<Eigen::SparseMatrix<double>> A; // 约束矩阵
};