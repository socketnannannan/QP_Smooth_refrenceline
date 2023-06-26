#include "egoVehicle.h"
#include "planner.h"

EgoVehicle::EgoVehicle(double v_target) {
  iter_qp_flag = 0;
  BornTime_ = chrono::system_clock::now();
  VTarget_ = v_target;
  StartPoint_ = make_tuple(right_lane_x[0], right_lane_y[0], 0, 0, 0, 0,
                           road_theta[0], 0);
  for (int i = 0; i < 50; i++) {
    LocalTrajectory_.push_back(StartPoint_);
  }
}

void EgoVehicle::GetState(Scenario &scenario) {
  // 当前时刻主车的信息, 不管qp如何迭代，一定是从当前点出发的
  ego_x_now = get<0>(LocalTrajectory_[0]);
  ego_y_now = get<1>(LocalTrajectory_[0]);
  ego_vx_now = get<2>(LocalTrajectory_[0]);
  ego_vy_now = get<3>(LocalTrajectory_[0]);
  ego_ax_now = get<4>(LocalTrajectory_[0]);
  ego_ay_now = get<5>(LocalTrajectory_[0]);
  ego_theta_now = get<6>(LocalTrajectory_[0]);
  ego_index_now = get<7>(LocalTrajectory_[0]);
  cout << "sudu : " << ego_vx_now << "," << ego_vy_now << endl;

  // 根据主车当前的状态刷新场景
  scenario.RefreshScenario(ego_index_now, VTarget_);

  // // 主车规划---test, 直接以目标速度进行规划以测试当前场景框架
  // double interVal = VTarget_ * 0.1; // 每0.1s行进的路程
  // double temp_interVal = 0;
  // double flag_index = ego_index_now;
  // // 未来5s的局部轨迹, 每一轮都是从当前位置的road_index开始规划
  // for (int i = 0; i < 50; i++) {
  //   tuple<double, double, double, double, double, double, double, int>
  //       PointPlan;
  //   PointPlan = make_tuple(0, 0, 0, 0, 0, 0, 0, 0);
  //   temp_interVal += interVal;
  //   while (temp_interVal >= 1) {
  //     flag_index++;
  //     temp_interVal -= 1;
  //   }
  //   // 在当前点所在的[flag_index, flag_index +
  //   1]的区间按照temp_interVal线性插值 get<7>(PointPlan) = flag_index;
  //   get<0>(PointPlan) = (1 - temp_interVal) * right_lane_x[flag_index] +
  //                       temp_interVal * right_lane_x[flag_index + 1];
  //   get<1>(PointPlan) = (1 - temp_interVal) * right_lane_y[flag_index] +
  //                       temp_interVal * right_lane_y[flag_index + 1];
  //   get<6>(PointPlan) = (1 - temp_interVal) * road_theta[flag_index] +
  //                       temp_interVal * road_theta[flag_index + 1];
  //   get<2>(PointPlan) = VTarget_ * cos(get<6>(PointPlan));
  //   get<3>(PointPlan) = VTarget_ * sin(get<6>(PointPlan));
  //   LocalTrajectory_.push_back(PointPlan);
  // }
}