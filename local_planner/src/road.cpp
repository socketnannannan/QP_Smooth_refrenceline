#include "road.h"

vector<double> left_road_x;
vector<double> left_road_y;
vector<double> right_road_x;
vector<double> right_road_y;
vector<double> center_road_x;
vector<double> center_road_y;
vector<double> left_lane_x;
vector<double> left_lane_y;
vector<double> right_lane_x;
vector<double> right_lane_y;
vector<double> road_theta;
vector<double> road_f_vector_cos;
vector<double> road_f_vector_sin;
vector<double> acc_lat_max;
vector<double> acc_forward_max;
vector<double> acc_backward_max;

void ReadCenterRoad(const char *filepath) {
  ifstream inFile(filepath, ios::in);
  string line, position;
  while (getline(inFile, line)) {
    istringstream sin(line);
    // 车道左边界点
    getline(sin, position, ',');
    left_road_x.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    left_road_y.emplace_back(atof(position.c_str()));
    // 车道右边界点
    getline(sin, position, ',');
    right_road_x.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    right_road_y.emplace_back(atof(position.c_str()));
    // 两车道间的车道线点
    getline(sin, position, ',');
    center_road_x.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    center_road_y.emplace_back(atof(position.c_str()));
    // 左侧车道中心线点
    getline(sin, position, ',');
    left_lane_x.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    left_lane_y.emplace_back(atof(position.c_str()));
    // 右侧车道中心线点
    getline(sin, position, ',');
    right_lane_x.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    right_lane_y.emplace_back(atof(position.c_str()));
    // 每一组点对应的theta, 切向量f_vector
    getline(sin, position, ',');
    road_theta.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    road_f_vector_cos.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    road_f_vector_sin.emplace_back(atof(position.c_str()));
  }
  // 车道回环处不要太跳跃
  left_road_y.back() = 3;
  right_road_y.back() = -3;
  center_road_y.back() = 0;
  left_lane_y.back() = 1.5;
  right_lane_y.back() = -1.5;
  inFile.close();
}

void AccInit(const char *filepath) {
  ifstream inFile(filepath, ios::in);
  string line, position;
  while (getline(inFile, line)) {
    istringstream sin(line);
    getline(sin, position, ',');
    acc_lat_max.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    acc_forward_max.emplace_back(atof(position.c_str()));
    getline(sin, position, ',');
    acc_backward_max.emplace_back(atof(position.c_str()));
  }
  inFile.close();
}