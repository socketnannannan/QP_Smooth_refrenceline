#pragma once
#include "matplotlibcpp.h"
#include "scenario.h"
#include <map>
#include <thread>
#include <tuple>

namespace plt = matplotlibcpp;

class EgoVehicle {
public:
  EgoVehicle(double v_target);
  ~EgoVehicle() = default;
  void GetState(Scenario &scenario);

  // 局部轨迹(x, y, vx, vy, ax, ay, theta, road_index)
  // 这里加入road_index的目的是方便做局部轨迹与参考线车道的匹配计算,
  // 这样可以省略每个点的暴力搜索匹配算法
  vector<tuple<double, double, double, double, double, double, double, int>>
      LocalTrajectory_;
  // 期望速度
  double VTarget_;
  // 创建时机
  chrono::time_point<chrono::system_clock> BornTime_;
  // 主车起点
  tuple<double, double, double, double, double, double, double, int>
      StartPoint_;
  // 主车当前的状态
  double ego_x_now;
  double ego_y_now;
  double ego_vx_now;
  double ego_vy_now;
  double ego_ax_now;
  double ego_ay_now;
  double ego_theta_now;
  int ego_index_now;

  int qp_iterations = 2;
  int iter_qp_flag;
};