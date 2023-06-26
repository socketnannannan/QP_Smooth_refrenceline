#include "planner.h"

// 把上一时刻的规划结果作为这一时刻的暖启动
void Planner_MPC::Initialization(EgoVehicle &ego) {
  LastLocalTrajectory_.clear();
  warm_start_.clear();
  LastLocalTrajectory_.insert(LastLocalTrajectory_.end(),
                              ego.LocalTrajectory_.begin() + 1,
                              ego.LocalTrajectory_.end());
  LastLocalTrajectory_.push_back(ego.LocalTrajectory_.back());
  for (int i = 0; i < predict_step_; i++) {
    warm_start_.push_back(get<0>(LastLocalTrajectory_[i]));
    warm_start_.push_back(get<1>(LastLocalTrajectory_[i]));
    warm_start_.push_back(get<2>(LastLocalTrajectory_[i]));
    warm_start_.push_back(get<3>(LastLocalTrajectory_[i]));
    warm_start_.push_back(get<4>(LastLocalTrajectory_[i]));
    warm_start_.push_back(get<5>(LastLocalTrajectory_[i]));
  }
  ego.LocalTrajectory_.clear();
}

void Planner_MPC::planner(Scenario &scenario, EgoVehicle &ego) {
  Q.clear();
  Q.push_back(CostFunction(scenario, ego));
  ConstriantFunction(scenario, ego);
  Solution(ego);
}

Eigen::SparseMatrix<double> Planner_MPC::CostFunction(Scenario &scenario,
                                                      EgoVehicle &ego) {

  Eigen::SparseMatrix<double> q(input_scale_, input_scale_);
  Eigen::VectorXd p = Eigen::VectorXd::Zero(input_scale_);
  // //                 1.1法向投影项
  // for (int i = 0; i < predict_step_; i++) {
  //   p(step_scale_ * i + 0) =
  //       -road_f_vector_cos[get<7>(LastLocalTrajectory_[i])];
  //   p(step_scale_ * i + 1) =
  //       -road_f_vector_sin[get<7>(LastLocalTrajectory_[i])];
  // }
  // // p(step_scale_ * 49 + 0) =
  // //     -0.1 * road_f_vector_cos[get<7>(LastLocalTrajectory_[49])];
  // // p(step_scale_ * 49 + 1) =
  // //     -0.1 * road_f_vector_sin[get<7>(LastLocalTrajectory_[49])];
  //                  1.2全局参考线项
  for (int i = 0; i < predict_step_; i++) {
    Eigen::Vector2d left_point(right_lane_x[get<7>(LastLocalTrajectory_[i])],
                               right_lane_y[get<7>(LastLocalTrajectory_[i])]);

    q.coeffRef(step_scale_ * i + 0, step_scale_ * i + 0) = 2;
    q.coeffRef(step_scale_ * i + 1, step_scale_ * i + 1) = 2;
    p(step_scale_ * i + 0) =
        -(right_lane_x[get<7>(LastLocalTrajectory_[i]) + 1] +
          right_lane_x[get<7>(LastLocalTrajectory_[i]) + 1]);
    p(step_scale_ * i + 1) =
        -(right_lane_y[get<7>(LastLocalTrajectory_[i]) + 1] +
          right_lane_y[get<7>(LastLocalTrajectory_[i]) + 1]);
  }
  //                  1.3控制量惩罚项
  for (int i = 0; i < predict_step_ - 1; i++) {
    // axi ayi的平方项
    q.coeffRef(step_scale_ * i + 4, step_scale_ * i + 4) = 2 * weight_R_;
    q.coeffRef(step_scale_ * i + 5, step_scale_ * i + 5) = 2 * weight_R_;
    // axi axi+1的交叉项 ayi ayi+1的交叉项
    q.coeffRef(step_scale_ * i + 4, step_scale_ * (i + 1) + 4) = -weight_R_;
    q.coeffRef(step_scale_ * (i + 1) + 4, step_scale_ * i + 4) = -weight_R_;
    q.coeffRef(step_scale_ * i + 5, step_scale_ * (i + 1) + 5) = -weight_R_;
    q.coeffRef(step_scale_ * (i + 1) + 5, step_scale_ * i + 5) = -weight_R_;
  }
  q.coeffRef(4, 4) = weight_R_;
  q.coeffRef(5, 5) = weight_R_;
  q.coeffRef(step_scale_ * (predict_step_ - 1) + 4,
             step_scale_ * (predict_step_ - 1) + 4) = weight_R_;
  q.coeffRef(step_scale_ * (predict_step_ - 1) + 5,
             step_scale_ * (predict_step_ - 1) + 5) = weight_R_;
  gradient.clear();
  gradient.emplace_back(p);
  return q;
}

void Planner_MPC::ConstriantFunction(Scenario &scenario, EgoVehicle &ego) {
  Eigen::SparseMatrix<double> a(eq_constraint_number_ + ineq_constraint_number_,
                                input_scale_);
  Eigen::VectorXd LowerBound(eq_constraint_number_ + ineq_constraint_number_);
  LowerBound.fill(-INF);
  Eigen::VectorXd UpperBound(eq_constraint_number_ + ineq_constraint_number_);
  UpperBound.fill(INF);
  // 1.等式约束
  //            1.1 线性模型xi+1 = Axi + Bu

  // 确保每次轨迹不管迭代到多少步，都是从当前状态开始规划的
  Eigen::Matrix4d Ad;
  Ad << 1, 0, dt_, 0, 0, 1, 0, dt_, 0, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix<double, 4, 2> Bd;
  Bd << dt_ * dt_ / 2, 0, 0, dt_ * dt_ / 2, dt_, 0, 0, dt_;
  a.coeffRef(0, 0) = 1;
  LowerBound(0) = get<0>(LastLocalTrajectory_[0]);
  UpperBound(0) = get<0>(LastLocalTrajectory_[0]);

  a.coeffRef(1, 1) = 1;
  LowerBound(1) = get<1>(LastLocalTrajectory_[0]);
  UpperBound(1) = get<1>(LastLocalTrajectory_[0]);

  a.coeffRef(2, 2) = 1;
  LowerBound(2) = get<2>(LastLocalTrajectory_[0]);
  UpperBound(2) = get<2>(LastLocalTrajectory_[0]);

  a.coeffRef(3, 3) = 1;
  LowerBound(3) = get<3>(LastLocalTrajectory_[0]);
  UpperBound(3) = get<3>(LastLocalTrajectory_[0]);
  // 确保每次轨迹不管迭代到多少步，都是从当前状态开始规划的

  for (int i = 1; i < predict_step_; i++) {
    // 线性模型的第一行 pxi = pxi-1 + dt * vxi-1 + dt^2/2 * axi
    a.coeffRef(4 * i + 0, step_scale_ * i + 0) = 1; // pxi
    a.coeffRef(4 * i + 0, step_scale_ * i + 4) =
        -Bd.coeff(0, 0); //-dt^2/2 * axi
    a.coeffRef(4 * i + 0, step_scale_ * (i - 1) + 0) = -Ad.coeff(0, 0); //-pxi-1
    a.coeffRef(4 * i + 0, step_scale_ * (i - 1) + 2) =
        -Ad.coeff(0, 2); //-dt * vxi-1
    LowerBound(4 * i + 0) = 0;
    UpperBound(4 * i + 0) = 0;
    // 线性模型的第二行 pyi = pyi-1 + dt * vyi-1 + dt^2/2 * ayi
    a.coeffRef(4 * i + 1, step_scale_ * i + 1) = 1; // pyi
    a.coeffRef(4 * i + 1, step_scale_ * i + 5) =
        -Bd.coeff(1, 1); //-dt^2/2 * ayi
    a.coeffRef(4 * i + 1, step_scale_ * (i - 1) + 1) = -Ad.coeff(1, 1); //-pyi-1
    a.coeffRef(4 * i + 1, step_scale_ * (i - 1) + 3) =
        -Ad.coeff(1, 3); //-dt * vyi-1
    LowerBound(4 * i + 1) = 0;
    UpperBound(4 * i + 1) = 0;
    // 线性模型的第三行 vxi = vxi-1 + dt * axi
    a.coeffRef(4 * i + 2, step_scale_ * i + 2) = 1;               // vxi
    a.coeffRef(4 * i + 2, step_scale_ * i + 4) = -Bd.coeff(2, 0); //-dt * axi
    a.coeffRef(4 * i + 2, step_scale_ * (i - 1) + 2) = -Ad.coeff(2, 2); //-vxi-1
    LowerBound(4 * i + 2) = 0;
    UpperBound(4 * i + 2) = 0;
    // 线性模型的第四行 vyi = vyi-1 + dt * ayi
    a.coeffRef(4 * i + 3, step_scale_ * i + 3) = 1;               // vyi
    a.coeffRef(4 * i + 3, step_scale_ * i + 5) = -Bd.coeff(3, 1); //-dt*ayi
    a.coeffRef(4 * i + 3, step_scale_ * (i - 1) + 3) = -Ad.coeff(3, 3); //-vyi-1
    LowerBound(4 * i + 3) = 0;
    UpperBound(4 * i + 3) = 0;
  }
  //            1.2 速度、加速度约束
  for (int i = 0; i < predict_step_; i++) {
    // vx
    a.coeffRef(eq_constraint_number_ + 6 * i + 0, step_scale_ * i + 2) = 1;
    if (road_f_vector_cos[get<7>(LastLocalTrajectory_[i])] >= 0) {
      LowerBound(eq_constraint_number_ + 6 * i + 0) = 0;
      UpperBound(eq_constraint_number_ + 6 * i + 0) =
          ego.VTarget_ * road_f_vector_cos[get<7>(LastLocalTrajectory_[i])];
    } else {
      LowerBound(eq_constraint_number_ + 6 * i + 0) =
          ego.VTarget_ * road_f_vector_cos[get<7>(LastLocalTrajectory_[i])];
      UpperBound(eq_constraint_number_ + 6 * i + 0) = 0;
    }
    // LowerBound(eq_constraint_number_ + 6 * i + 0) = -15.6;
    // UpperBound(eq_constraint_number_ + 6 * i + 0) = 15.6;
    //  vy
    a.coeffRef(eq_constraint_number_ + 6 * i + 1, step_scale_ * i + 3) = 1;
    if (road_f_vector_sin[get<7>(LastLocalTrajectory_[i])] >= 0) {
      // 保证在平直道路时车辆依然能够具备充足的侧向速度，维持换道的能力
      if (road_f_vector_sin[get<7>(LastLocalTrajectory_[i])] < 0.4) {
        LowerBound(eq_constraint_number_ + 6 * i + 1) = -0.5 * ego.VTarget_;
        UpperBound(eq_constraint_number_ + 6 * i + 1) = 0.5 * ego.VTarget_;
      } else {
        LowerBound(eq_constraint_number_ + 6 * i + 1) = 0;
        UpperBound(eq_constraint_number_ + 6 * i + 1) =
            ego.VTarget_ * road_f_vector_sin[get<7>(LastLocalTrajectory_[i])];
      }
    } else {
      if (road_f_vector_sin[get<7>(LastLocalTrajectory_[i])] > -0.4) {
        LowerBound(eq_constraint_number_ + 6 * i + 1) = -0.5 * ego.VTarget_;
        UpperBound(eq_constraint_number_ + 6 * i + 1) = 0.5 * ego.VTarget_;
      } else {
        LowerBound(eq_constraint_number_ + 6 * i + 1) =
            ego.VTarget_ * road_f_vector_sin[get<7>(LastLocalTrajectory_[i])];
        UpperBound(eq_constraint_number_ + 6 * i + 1) = 0;
      }
    }
    //  ax
    a.coeffRef(eq_constraint_number_ + 6 * i + 2, step_scale_ * i + 4) = 1;
    LowerBound(eq_constraint_number_ + 6 * i + 2) = -8;
    UpperBound(eq_constraint_number_ + 6 * i + 2) = 8;
    // ay
    a.coeffRef(eq_constraint_number_ + 6 * i + 3, step_scale_ * i + 5) = 1;
    LowerBound(eq_constraint_number_ + 6 * i + 3) = -8;
    UpperBound(eq_constraint_number_ + 6 * i + 3) = 8;

    //            1.3 边界约束
    // 左边界
    a.coeffRef(eq_constraint_number_ + 6 * i + 4, step_scale_ * i + 0) =
        -road_f_vector_sin[get<7>(
            LastLocalTrajectory_[i])]; // xi * -sin(road_index)
    a.coeffRef(eq_constraint_number_ + 6 * i + 4,
               step_scale_ * i + 1) =
        road_f_vector_cos[get<7>(
            LastLocalTrajectory_[i])]; // yi * cos(road_index)
    LowerBound(eq_constraint_number_ + 6 * i + 4) = -1 * INF;
    UpperBound(eq_constraint_number_ + 6 * i + 4) =
        -road_f_vector_sin[get<7>(LastLocalTrajectory_[i])] *
            left_lane_x[get<7>(LastLocalTrajectory_[i])] +
        road_f_vector_cos[get<7>(LastLocalTrajectory_[i])] *
            left_lane_y[get<7>(LastLocalTrajectory_[i])];
    // 右边界
    a.coeffRef(eq_constraint_number_ + 6 * i + 5,
               step_scale_ * i + 0) =
        road_f_vector_sin[get<7>(
            LastLocalTrajectory_[i])]; // xi * -sin(road_index)
    a.coeffRef(eq_constraint_number_ + 6 * i + 5,
               step_scale_ * i + 1) =
        -road_f_vector_cos[get<7>(
            LastLocalTrajectory_[i])]; // yi * cos(road_index)
    LowerBound(eq_constraint_number_ + 6 * i + 5) = -1 * INF;
    UpperBound(eq_constraint_number_ + 6 * i + 5) =
        road_f_vector_sin[get<7>(LastLocalTrajectory_[i])] *
            right_lane_x[get<7>(LastLocalTrajectory_[i])] -
        road_f_vector_cos[get<7>(LastLocalTrajectory_[i])] *
            right_lane_y[get<7>(LastLocalTrajectory_[i])];
  }
  //            1.4 障碍物约束(直接引入到边界约束进行处理)
  for (auto obs : scenario.obstacles_) {
    // 如果障碍物在左边, 则影响的是左边界; 如果障碍物在右边, 则影响的是右边界
    auto t_now = chrono::system_clock::now();
    chrono::milliseconds used =
        chrono::duration_cast<chrono::milliseconds>(t_now - obs.BornTime_);
    int obs_now_index = floor(used.count() / 100);
    for (int i = 0; i < predict_step_; i++) {
      double dist_now = sqrt(pow((get<0>(LastLocalTrajectory_[i]) -
                                  obs.PositionX_[obs_now_index + i]),
                                 2) +
                             pow((get<1>(LastLocalTrajectory_[i]) -
                                  obs.PositionY_[obs_now_index + i]),
                                 2));
      // 距离障碍物的最小距离为8m，实际上减去车身，仅余4m
      if (dist_now < 6) {
        int temp_step = 4;
        if (i < 2 * temp_step) {
          for (int j = temp_step; j <= i + 2; j++) {
            if (obs.Lane_ == 0)
              UpperBound(eq_constraint_number_ + 6 * j + 4) =
                  -road_f_vector_sin[get<7>(LastLocalTrajectory_[j])] *
                      right_lane_x[get<7>(LastLocalTrajectory_[j])] +
                  road_f_vector_cos[get<7>(LastLocalTrajectory_[j])] *
                      right_lane_y[get<7>(LastLocalTrajectory_[j])];
            if (obs.Lane_ == 1)
              UpperBound(eq_constraint_number_ + 6 * j + 5) =
                  road_f_vector_sin[get<7>(LastLocalTrajectory_[j])] *
                      left_lane_x[get<7>(LastLocalTrajectory_[j])] -
                  road_f_vector_cos[get<7>(LastLocalTrajectory_[j])] *
                      left_lane_y[get<7>(LastLocalTrajectory_[j])];
          }
        } else if (i > 47) {
          for (int j = i - 2; j <= predict_step_ - 1; j++) {
            if (obs.Lane_ == 0)
              UpperBound(eq_constraint_number_ + 6 * j + 4) =
                  -road_f_vector_sin[get<7>(LastLocalTrajectory_[j])] *
                      right_lane_x[get<7>(LastLocalTrajectory_[j])] +
                  road_f_vector_cos[get<7>(LastLocalTrajectory_[j])] *
                      right_lane_y[get<7>(LastLocalTrajectory_[j])];
            if (obs.Lane_ == 1)
              UpperBound(eq_constraint_number_ + 6 * j + 5) =
                  road_f_vector_sin[get<7>(LastLocalTrajectory_[i])] *
                      left_lane_x[get<7>(LastLocalTrajectory_[i])] -
                  road_f_vector_cos[get<7>(LastLocalTrajectory_[i])] *
                      left_lane_y[get<7>(LastLocalTrajectory_[i])];
          }
        } else {
          for (int j = i - 2; j <= i + 2; j++) {
            if (obs.Lane_ == 0)
              UpperBound(eq_constraint_number_ + 6 * j + 4) =
                  -road_f_vector_sin[get<7>(LastLocalTrajectory_[j])] *
                      right_lane_x[get<7>(LastLocalTrajectory_[j])] +
                  road_f_vector_cos[get<7>(LastLocalTrajectory_[j])] *
                      right_lane_y[get<7>(LastLocalTrajectory_[j])];
            if (obs.Lane_ == 1)
              UpperBound(eq_constraint_number_ + 6 * j + 5) =
                  road_f_vector_sin[get<7>(LastLocalTrajectory_[j])] *
                      left_lane_x[get<7>(LastLocalTrajectory_[j])] -
                  road_f_vector_cos[get<7>(LastLocalTrajectory_[j])] *
                      left_lane_y[get<7>(LastLocalTrajectory_[j])];
          }
        }
      }
    }
  }
  //            1.5 终端速度约束: 保证轨迹的可控性，在求解失败时保障车辆的安全
  a.coeffRef(eq_constraint_number_ + 6 * predict_step_ + 0,
             step_scale_ * 49 + 2) = 1;
  LowerBound(eq_constraint_number_ + 6 * predict_step_ + 0) = 0;
  UpperBound(eq_constraint_number_ + 6 * predict_step_ + 0) = 0;
  a.coeffRef(eq_constraint_number_ + 6 * predict_step_ + 1,
             step_scale_ * 49 + 3) = 1;
  LowerBound(eq_constraint_number_ + 6 * predict_step_ + 1) = 0;
  UpperBound(eq_constraint_number_ + 6 * predict_step_ + 1) = 0;
  A.clear();
  bound.clear();
  A.emplace_back(a);
  bound.emplace_back(LowerBound);
  bound.emplace_back(UpperBound);
}

bool Planner_MPC::Solution(EgoVehicle &ego) {
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(false);
  Eigen::Matrix<double, 301, 1> mat = Eigen::Matrix<double, 301, 1>::Zero();

  // 超车积极性稍高
  for (int i = 0; i < 301; i++) {
    mat(i, 0) = warm_start_[i];
  }

  // //这种启动值超车积极性更低
  // for (int i = 0; i < 6; i++) {
  //   mat(i, 0) = warm_start_[i];
  // }
  // for (int i = 6; i < 301; i++) {
  //   mat(i, 0) = warm_start_[i - 6];
  // }
  solver.data()->setNumberOfVariables(input_scale_);
  solver.data()->setNumberOfConstraints(eq_constraint_number_ +
                                        ineq_constraint_number_);

  if (!solver.data()->setHessianMatrix(Q[0]))
    return false;
  if (!solver.data()->setGradient(gradient[0]))
    return false;
  if (!solver.data()->setLinearConstraintsMatrix(A[0]))
    return false;
  if (!solver.data()->setLowerBound(bound[0]))
    return false;
  if (!solver.data()->setUpperBound(bound[1]))
    return false;
  Eigen::VectorXd v =
      Eigen::Map<Eigen::VectorXd>(warm_start_.data(), warm_start_.size());
  if (!solver.initSolver())
    return false;
  if (!solver.setPrimalVariable(mat))
    return false;
  Eigen::VectorXd Solution;
  if (!solver.solve())
    return false;
  cout << "problem solved" << endl;
  Solution = solver.getSolution();
  ResultToTrajectory(Solution, ego);
  return true;
}

void Planner_MPC::ResultToTrajectory(Eigen::VectorXd &solution,
                                     EgoVehicle &ego) {
  // warm-start和local-trajectory
  warm_start_.clear();
  LastLocalTrajectory_.clear();
  for (int i = 0; i < input_scale_; i++) {
    warm_start_.push_back(solution(i));
  }

  tuple<double, double, double, double, double, double, double, int> PointPlan;
  double temp = INF;
  int flag_index = ego.ego_index_now;
  for (int i = 0; i < 10; i++) {
    double distance =
        sqrt(pow(solution(0) - center_road_x[ego.ego_index_now + i], 2) +
             pow(solution(1) - center_road_y[ego.ego_index_now + i], 2));
    if (distance < temp) {
      temp = distance;
      flag_index = ego.ego_index_now + i;
    }
  }
  get<7>(PointPlan) = flag_index;
  get<0>(PointPlan) = solution(0);
  get<1>(PointPlan) = solution(1);
  get<2>(PointPlan) = solution(2);
  get<3>(PointPlan) = solution(3);
  get<4>(PointPlan) = solution(4);
  get<5>(PointPlan) = solution(5);
  get<6>(PointPlan) = atan2(get<3>(PointPlan), get<2>(PointPlan));
  LastLocalTrajectory_.push_back(PointPlan);
  for (int i = 1; i < predict_step_; i++) {
    double temp1 = INF;
    int flag_index1 = get<7>(LastLocalTrajectory_[i - 1]);
    int flag_add = 0;
    for (int j = 0; j < 10; j++) {
      double distance1 = sqrt(
          pow(solution(step_scale_ * i + 0) - center_road_x[flag_index1 + j],
              2) +
          pow(solution(step_scale_ * i + 1) - center_road_y[flag_index1 + j],
              2));
      if (distance1 < temp1) {
        temp1 = distance1;
        flag_add = j;
      }
    }
    get<7>(PointPlan) = flag_index1 + flag_add;
    get<0>(PointPlan) = solution(step_scale_ * i + 0);
    get<1>(PointPlan) = solution(step_scale_ * i + 1);
    get<2>(PointPlan) = solution(step_scale_ * i + 2);
    get<3>(PointPlan) = solution(step_scale_ * i + 3);
    get<4>(PointPlan) = solution(step_scale_ * i + 4);
    get<5>(PointPlan) = solution(step_scale_ * i + 5);
    get<6>(PointPlan) = atan2(get<3>(PointPlan), get<2>(PointPlan));
    LastLocalTrajectory_.push_back(PointPlan);
  }
}