#pragma once
#include "road.h"
#include <chrono>
#include <ctime>
#include <random>

class Obstacle {
  // 规划算法需要障碍物从当前时刻算起未来5s之内的数据
  // 障碍物的轨迹可以使用vx, vy计算得到终点，然后插值得到轨迹序列(0.1s间隔)
public:
  Obstacle(int ego_road_index,
           double ego_cur_v); // 传入ego_vehicle信息，依据此生成障碍物
  ~Obstacle() = default;
  vector<double> PositionX_;
  vector<double> PositionY_;
  vector<double> Head_;
  vector<int> Index_; // 记录下每个point此时在road中对应的index
  double V_;
  chrono::time_point<chrono::system_clock> BornTime_;

  int Lane_; // 障碍物的lane, 0左1右, 用于检测障碍物生成时是否会发生碰撞
};

// 障碍物轨迹
class Scenario {
public:
  Scenario() { cout << "scenario born" << endl; } // 生成场景
  ~Scenario() = default;
  void RefreshScenario(int ego_road_index, double ego_cur_v); // 刷新场景
  void GenerateNextScenario(int ego_road_index, double ego_cur_v); // 生成场景
  bool IsOverScenario(int ego_raod_index); // 检查当前场景是否结束
  vector<Obstacle> obstacles_;
};