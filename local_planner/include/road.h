#pragma once
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// 车道左边界点
extern vector<double> left_road_x;
extern vector<double> left_road_y;
// 两车道间的车道线点
extern vector<double> center_road_x;
extern vector<double> center_road_y;
// 车道右边界点
extern vector<double> right_road_x;
extern vector<double> right_road_y;
// 左侧车道中心线点
extern vector<double> left_lane_x;
extern vector<double> left_lane_y;
// 右侧车道中心线点
extern vector<double> right_lane_x;
extern vector<double> right_lane_y;
// 每一组点对应的theta, 切向量f_vector
extern vector<double> road_theta;
extern vector<double> road_f_vector_cos;
extern vector<double> road_f_vector_sin;

extern vector<double> acc_lat_max;
extern vector<double> acc_forward_max;
extern vector<double> acc_backward_max;

// 将地图中心线读取到road
void ReadCenterRoad(const char *filepath);

// 读取车辆的加速度圆
void AccInit(const char *filepath);