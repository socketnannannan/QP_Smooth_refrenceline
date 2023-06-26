#include "scenario.h"

Obstacle::Obstacle(int ego_road_index, double ego_cur_v) {
  BornTime_ = chrono::system_clock::now();

  // 根据主车的速度随机生成障碍物的速度
  random_device rd;
  mt19937 eng(rd());
  uniform_real_distribution<double> dist_v(    //障碍物速度范围
      0.1 * ego_cur_v,
      0.4 * ego_cur_v); // 0.5ego_cur_v~ego_cur_v之间实数均匀分布
  double V_ = dist_v(eng);
  double interVal = V_ * 0.1; // 每0.1s行进的路程

  // 障碍物的出生位置--100m~150m之间, 以及其未来60s内的轨迹
  uniform_int_distribution<int> dist_pos(ego_road_index + 40,
                                         ego_road_index + 80);
  int random_index = dist_pos(eng); // 选择了出生的Index
  uniform_int_distribution<int> dist_lane(0, 1);     //两条车道  0和1
  Lane_ = dist_lane(eng); // 选择了出生的车道
  if (Lane_ == 0) {
    PositionX_.push_back(left_lane_x[random_index]);
    PositionY_.push_back(left_lane_y[random_index]);
    Head_.push_back(road_theta[random_index]);
    Index_.push_back(random_index);
    double temp_interVal = 0;
    double flag_index = random_index;
    // 障碍物在当前速度下的60s轨迹
    for (int i = 0; i < 600; i++) {
      temp_interVal += interVal;
      while (temp_interVal >= 1) {
        flag_index++;
        temp_interVal -= 1;
      }
      // 在当前点所在的[flag_index, flag_index +
      // 1]的区间按照temp_interVal线性插值
      Index_.push_back(flag_index);
      PositionX_.push_back((1 - temp_interVal) * left_lane_x[flag_index] +
                           temp_interVal * left_lane_x[flag_index + 1]);
      PositionY_.push_back((1 - temp_interVal) * left_lane_y[flag_index] +
                           temp_interVal * left_lane_y[flag_index + 1]);
      Head_.push_back((1 - temp_interVal) * road_theta[flag_index] +
                      temp_interVal * road_theta[flag_index + 1]);
    }
  } else if (Lane_ == 1) {
    PositionX_.push_back(right_lane_x[random_index]);
    PositionY_.push_back(right_lane_y[random_index]);
    Head_.push_back(road_theta[random_index]);
    Index_.push_back(random_index);
    double temp_interVal = 0;
    double flag_index = random_index;
    // 障碍物在当前速度下的60s轨迹
    for (int i = 0; i < 600; i++) {
      temp_interVal += interVal;
      while (temp_interVal >= 1) {
        flag_index++;
        temp_interVal -= 1;
      }
      // 在当前点所在的[flag_index, flag_index +
      // 1]的区间按照temp_interVal线性插值
      Index_.push_back(flag_index);
      PositionX_.push_back((1 - temp_interVal) * right_lane_x[flag_index] +
                           temp_interVal * right_lane_x[flag_index + 1]);
      PositionY_.push_back((1 - temp_interVal) * right_lane_y[flag_index] +
                           temp_interVal * right_lane_y[flag_index + 1]);
      Head_.push_back((1 - temp_interVal) * road_theta[flag_index] +
                      temp_interVal * road_theta[flag_index + 1]);
    }
  }
}

// 检查是否需要生成下一个场景，若需要则生成
void Scenario::RefreshScenario(int ego_road_index, double ego_cur_v) {
  if (IsOverScenario(ego_road_index))
    GenerateNextScenario(ego_road_index, ego_cur_v);
}

// 产生下一个场景，并且生成的障碍物经过合理性检查
void Scenario::GenerateNextScenario(int ego_road_index, double ego_cur_v) {
  obstacles_.clear(); // 先清空上一个场景的障碍物
  random_device rd;
  mt19937 eng(rd());
  uniform_int_distribution<int> dist_num(1, 4);
  int obstacle_num = dist_num(eng); // 场景中随机生成1~5个障碍物
  chrono::steady_clock::time_point tbegin = chrono::steady_clock::now();
  chrono::steady_clock::time_point tend;
  chrono::milliseconds used;
  while (obstacles_.size() < obstacle_num) {
    // 障碍物生成时间不能超过10ms
    tend = chrono::steady_clock::now();
    used = chrono::duration_cast<std::chrono::milliseconds>(tend - tbegin);
    if (used.count() > 10)
      break;

    Obstacle obstacle(ego_road_index, ego_cur_v);
    if (obstacles_.size() == 0) {
      obstacles_.push_back(obstacle);
      continue;
    }
    bool flag = true;
    for (auto obs : obstacles_) {
      // 如果生成的障碍物在已有障碍物的前方，并且速度低了很多，则障碍物之间会碰撞，舍弃该障碍物
      if (obstacle.Lane_ == obs.Lane_ && obstacle.Index_[0] > obs.Index_[0] &&
          obstacle.V_ < 0.8 * obs.V_) {
        flag = false;
        break;
      }
      // 障碍物出现在了前后8m内
      if (obstacle.Index_[0] <= obs.Index_[0] + 10 &&
          obstacle.Index_[0] >= obs.Index_[0] - 10) {
        flag = false;
        break;
      }
    }
    // 障碍物合理则放入场景中
    if (flag)
      obstacles_.push_back(obstacle);
  }
  cout << "create: " << obstacles_.size() << " obstacle" << endl;
}

// 如果障碍物此时的road_index全部在ego的road_index后面，则表示当前场景ego车已经顺利通过
bool Scenario::IsOverScenario(int ego_road_index) {
  auto time_now = chrono::system_clock::now();
  for (auto obs : obstacles_) {
    chrono::milliseconds used =
        chrono::duration_cast<chrono::milliseconds>(time_now - obs.BornTime_);
    int obs_now_index = floor(used.count() / 100);
    if (obs.Index_[obs_now_index] + 8 > ego_road_index)
      return false;
  }
  return true;
}
