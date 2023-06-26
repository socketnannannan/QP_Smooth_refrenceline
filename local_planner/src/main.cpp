#include "egoVehicle.h"
#include "planner.h"
#include "road.h"
#include "scenario.h"
#include <map>

void Print(const Scenario &sce, const EgoVehicle &ego_car);

int main() {
  // 获取当前道路的中心线和加速度圆
  const char *file = "../map/map.csv";
  ReadCenterRoad(file);
  const char *file1 = "../map/acc_max.csv";
  AccInit(file1);

  Scenario sce;
  EgoVehicle ego_car(15);
  Planner_MPC local_planner;
  plt::figure_size(1200, 1200);
  while (true) {
    chrono::steady_clock::time_point tbegin = chrono::steady_clock::now();
    sce.RefreshScenario(get<7>(ego_car.LocalTrajectory_[0]), ego_car.VTarget_);
    ego_car.GetState(sce);
    local_planner.Initialization(ego_car);
    for (int i = 0; i < ego_car.qp_iterations; i++) {
      local_planner.planner(sce, ego_car);
    }
    ego_car.LocalTrajectory_ = local_planner.LastLocalTrajectory_;
    chrono::steady_clock::time_point tend = chrono::steady_clock::now();
    chrono::milliseconds used =
        chrono::duration_cast<chrono::milliseconds>(tend - tbegin);
    Print(sce, ego_car);
    if (used.count() > 100)
      continue;
    this_thread::sleep_for(chrono::milliseconds(100 - used.count()));
  }
  return 0;
}

// 将车辆的四个顶点围绕中心点旋转theta度
void rotate(vector<double> &ego_x, vector<double> &ego_y, double x, double y,
            double x_center, double y_center, double theta) {
  double rad = theta; // 弧度
  double d = 2.236;   // 计算点之间的距离
  double alpha =
      std::atan2(y - y_center, x - x_center); // 计算点 a 相对于点 b 的方位角
  alpha += rad;                               // 增加角度
  x = x_center + d * std::cos(alpha);         // 更新点 a 的 x 坐标
  y = y_center + d * std::sin(alpha);         // 更新点 a 的 y 坐标
  ego_x.push_back(x);
  ego_y.push_back(y);
}

void Print(const Scenario &sce, const EgoVehicle &ego_car) {   //打印出三条车道线
  // 打印车道
  plt::clf();
  plt::plot(center_road_x, center_road_y, "k--");
  plt::plot(left_road_x, left_road_y, "k");
  plt::plot(right_road_x, right_road_y, "k");

  // 打印当前的场景
  vector<double> ego_x;
  vector<double> ego_y;
  int ego_index_now = get<7>(ego_car.LocalTrajectory_[0]);
  double ego_x_now = get<0>(ego_car.LocalTrajectory_[0]);
  double ego_y_now = get<1>(ego_car.LocalTrajectory_[0]);
  double ego_theta_now = get<6>(ego_car.LocalTrajectory_[0]);
  rotate(ego_x, ego_y, ego_x_now - 2, ego_y_now - 1, ego_x_now, ego_y_now,
         ego_theta_now);
  rotate(ego_x, ego_y, ego_x_now - 2, ego_y_now + 1, ego_x_now, ego_y_now,
         ego_theta_now);
  rotate(ego_x, ego_y, ego_x_now + 2, ego_y_now + 1, ego_x_now, ego_y_now,
         ego_theta_now);
  rotate(ego_x, ego_y, ego_x_now + 2, ego_y_now - 1, ego_x_now, ego_y_now,
         ego_theta_now);
  map<string, string> keywords_ego;
  keywords_ego.insert(pair<string, string>("color", "blue"));
  plt::fill(ego_x, ego_y, keywords_ego);
  vector<double> ego_traj_x;
  vector<double> ego_traj_y;
  for (int i = 0; i < ego_car.LocalTrajectory_.size(); i++) {
    ego_traj_x.push_back(get<0>(ego_car.LocalTrajectory_[i]));
    ego_traj_y.push_back(get<1>(ego_car.LocalTrajectory_[i]));
  }
  plt::named_plot("mpc_trajectory", ego_traj_x, ego_traj_y, "r");

  for (auto obs : sce.obstacles_) {
    auto time_now = chrono::system_clock::now();
    chrono::milliseconds used =
        chrono::duration_cast<chrono::milliseconds>(time_now - obs.BornTime_);
    int obs_index_now = floor(used.count() / 100);
    vector<double> x;
    vector<double> y;
    rotate(x, y, obs.PositionX_[obs_index_now] - 2,
           obs.PositionY_[obs_index_now] - 1, obs.PositionX_[obs_index_now],
           obs.PositionY_[obs_index_now], obs.Head_[obs_index_now]);
    rotate(x, y, obs.PositionX_[obs_index_now] - 2,
           obs.PositionY_[obs_index_now] + 1, obs.PositionX_[obs_index_now],
           obs.PositionY_[obs_index_now], obs.Head_[obs_index_now]);
    rotate(x, y, obs.PositionX_[obs_index_now] + 2,
           obs.PositionY_[obs_index_now] + 1, obs.PositionX_[obs_index_now],
           obs.PositionY_[obs_index_now], obs.Head_[obs_index_now]);
    rotate(x, y, obs.PositionX_[obs_index_now] + 2,
           obs.PositionY_[obs_index_now] - 1, obs.PositionX_[obs_index_now],
           obs.PositionY_[obs_index_now], obs.Head_[obs_index_now]);
    map<string, string> keywords;
    keywords.insert(pair<string, string>("color", "black"));
    plt::fill(x, y, keywords);
  }

  // 轨迹的起点打印
  vector<double> plannning_x_start;
  vector<double> plannning_y_start;
  plannning_x_start.push_back(center_road_x[ego_car.ego_index_now] - 0.5);
  plannning_y_start.push_back(center_road_y[ego_car.ego_index_now] - 0.5);
  plannning_x_start.push_back(center_road_x[ego_car.ego_index_now] - 0.5);
  plannning_y_start.push_back(center_road_y[ego_car.ego_index_now] + 0.5);
  plannning_x_start.push_back(center_road_x[ego_car.ego_index_now] + 0.5);
  plannning_y_start.push_back(center_road_y[ego_car.ego_index_now] + 0.5);
  plannning_x_start.push_back(center_road_x[ego_car.ego_index_now] + 0.5);
  plannning_y_start.push_back(center_road_y[ego_car.ego_index_now] - 0.5);
  map<string, string> keywords_planning_start;
  keywords_planning_start.insert(pair<string, string>("color", "green"));
  plt::fill(plannning_x_start, plannning_y_start, keywords_planning_start);

  // 找到曲线的中心点
  plt::xlim(center_road_x[ego_car.ego_index_now + 50] - 60,
            center_road_x[ego_car.ego_index_now + 50] + 60);
  plt::ylim(right_road_y[ego_car.ego_index_now + 50] - 56,
            left_road_y[ego_car.ego_index_now + 50] + 56);
  plt::title("local planner");
  map<string, string> keywords_label;
  keywords_label.insert(pair<string, string>("loc", "upper left"));
  plt::legend(keywords_label);
  plt::pause(0.001);
}