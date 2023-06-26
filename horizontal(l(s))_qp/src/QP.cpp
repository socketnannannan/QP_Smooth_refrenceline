#include "QP.h"
#include <iostream>
using namespace std;

/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/* #include "modules/planning/math/piecewise_jerk/piecewise_jerk_problem.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h" */

/* PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init) */

namespace {
constexpr double kMaxVariableRange = 1.0e10;  //constexpr是C++11引入的关键字，用于声明一个编译时常量。
                                              //它可以应用于变量、函数、类型等，并且要求其值或表达式在编译时就能确定。
}  // namespace

PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init) {
        
  //CHECK_GE(num_of_knots, 2U);
  num_of_knots_ = num_of_knots;

  x_init_ = x_init;

  delta_s_ = delta_s;

  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
}

//求解各矩阵获取
OSQPData* PiecewiseJerkProblem::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  //CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  size_t kernel_dim = 3 * num_of_knots_;
  size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim;                 //A矩阵行维度
  data->m = num_affine_constraint;      //A矩阵列维度
  data->P = csc_to_triu(csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr)));
  //csc_to_triu((csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p)); //csc_to_triu可以将csc的矩阵转换为上三角矩阵
  data->q = CopyData(q);
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

OSQPSettings* PiecewiseJerkProblem::SolverDefaultSettings() { // 函数的声明返回一个指向 OSQPData 类型对象的指针。
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  //settings->verbose = FLAGS_enable_osqp_debug;
  settings->scaled_termination = true;
  return settings;
}

bool PiecewiseJerkProblem::Optimize(const int max_iter) {
  OSQPData* data = FormulateProblem();

  OSQPSettings* settings = SolverDefaultSettings();
  settings->max_iter = max_iter;

  OSQPWorkspace* osqp_work = nullptr;
  //osqp_work = osqp_setup(data, settings);

  osqp_setup(&osqp_work, data, settings);
  osqp_solve(osqp_work);

  auto status = osqp_work->info->status_val; 
  cout << status << endl;

  if (status < 0 || (status != 1 && status != 2)) {
    //AERROR << "failed optimization status:\t" << osqp_work->info->status;
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_work->solution == nullptr) {
    //AERROR << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
    ddx_.at(i) = osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
  }
  std::vector<int> numbers;
  for (int i = 1; i <= num_of_knots_; ++i) {
      numbers.push_back(i);
  }
  plt::clf();
  plt::plot(numbers, x_, "b");
  plt::figure();
  plt::plot(numbers, dx_, "g");
  plt::figure();
  plt::plot(numbers, ddx_, "r");
  plt::show();
  //plt::plot(global_path_x, global_path_y, "k--");

  // Cleanup
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);
  return true;
}


/*******************************************************************************
 * 
 * 1、计算 Hessian 矩阵
 * 2、转变成 OSQP求解器需要的素材
 * 
 * *****************************************************************************/
void PiecewiseJerkProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {

  const int n = static_cast<int>(num_of_knots_);  // 待平滑的点数
  const int num_of_variables = 3 * n;             // 决策变量的数目
  const int num_of_nonzeros = num_of_variables + (n - 1);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
/*   这行代码声明了一个名为columns的二维向量，其大小为num_of_variables。
  该向量用于存储决策变量的非零元素，每个变量对应一个内部向量。每个内部向量包含一对(c_int, c_float)，
  其中c_int表示行索引，c_float表示非零元素的值。
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables); */
  int value_index = 0;

  
  // x(i)^2 * (w_x + w_x_ref[i]), w_x_ref might be a uniform value for all x(i) or piecewise values for different x(i)
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, (weight_x_ + weight_x_ref_vec_[i]) /
                               (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // x(n-1)^2 * (w_x + w_x_ref[n-1] + w_end_x)
  // 末态单独处理
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_vec_[n - 1] + weight_end_state_[0]) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;


  // x(i)' ^2 * w_dx
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  } 
  // x(n-1)' ^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  (weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  // 在计算 l'' ^2 的权重时， 考虑了  l''' ^2 的权重
  // 其中 l''' = ( l''(i)-l''(i-1) ) / delta_s
  // l'''^2 = l''(i)^2 /delta_s^2   +  l''(i-1)^2 /delta_s^2  -  2 * l''(i)*l''(i-1) /delta_s^2
  auto delta_s_square = delta_s_ * delta_s_;
  // x(i)'' ^2  * (w_ddx + 2 * w_dddx / delta_s^2)
  // 所以每个 l''(i)^2 的权重有两部分组成，一个是w_ddx， 一个是w_dddx
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;


  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  // hession矩阵的 右下角这个 n*n的矩阵，除了对角线元素，它左下侧还有一排元素
  /***       |    o                  | 
   *         |         o             |
   *         |              o        |
   *         |              o  o     |
   *         |                 o  0  |
   * ***/ 
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    (-2.0 * weight_dddx_ / delta_s_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  //CHECK_EQ(value_index, num_of_nonzeros);

  // 转换成csc_matrix的形式
  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second * 2.0); // P_data来记录 hession矩阵的元素        
      P_indices->push_back(row_data_pair.first);     // P_indices来记录各个元素的行号               
      ++ind_p;                                                                                           
    }                                                                                                    
  }                                                                                                      
  P_indptr->push_back(ind_p);                                                                            
}



/*******************************************************************************
 * g矩阵
 * 目标函数中的 (l-l_ref)^2 = l^2 - 2*l_ref*l + l_ref^2
 * l_ref^2是个非负实数，对梯度没贡献可以忽略，g矩阵中就只剩下 -2*l_ref*l
 * 
 * *****************************************************************************/
void PiecewiseJerkProblem::CalculateOffset(std::vector<c_float>* q) {
  //CHECK_NOTNULL(q);  //首先使用 CHECK_NOTNULL 宏来检查指针 q 是否为 nullptr，以确保传入的指针是有效的。
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam, 0.0);  //使用 resize 函数将指针 q 指向的向量的大小调整为 kNumParam，并将所有元素初始化为 0.0

  if (has_x_ref_) {
    // l^2+(l-ref)^2 拆开项中的 -2ref*i
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_vec_.at(i) * x_ref_[i] / scale_factor_[0];
    }
  }

  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}




/*******************************************************************************
 * 
 * 计算约束条件的 A 矩阵  和上下边界 lower_bounds    upper_bounds 
 * 
 * *****************************************************************************/
void PiecewiseJerkProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 3N个    上下界约束
  // 3(N-1)  连续性约束
  // 3个     初始约束
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;                                // 决策变量个数
  const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3; // 约束条件的个数
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  std::vector<std::vector<std::pair<c_int, c_float>>> variables(num_of_variables);

  int constraint_index = 0;

  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }
  //CHECK_EQ(constraint_index, num_of_variables);

  // x" 加速度约束
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(constraint_index, -1.0);
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) = dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
  }

  // x' 速度连续性约束
  // x(i+1)' - x(i)' -  0.5*delta_s *x(i)'' -  0.5*delta_s *x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i    ].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    variables[n + i + 1].emplace_back(constraint_index,  1.0 * scale_factor_[2]);
    variables[2 * n + i].emplace_back(constraint_index, -0.5 * delta_s_ * scale_factor_[1]);
    variables[2*n + i+1].emplace_back(constraint_index, -0.5 * delta_s_ * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x 位置连续性约束
  // x(i+1) =  x(i) + delta_s * x(i)' + 1/3* delta_s^2 * x(i)'' + 1/6* delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables[i        ].emplace_back(constraint_index, -1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[i   + 1  ].emplace_back(constraint_index,  1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[n   + i  ].emplace_back(constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
    variables[2*n + i  ].emplace_back(constraint_index, -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    variables[2*n + i+1].emplace_back(constraint_index, -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // 初始状态约束
  // constrain on x_init、x'_init、x"_init
  variables[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  variables[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  variables[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  //CHECK_EQ(constraint_index, num_of_constraints);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      // coefficient
      A_data->push_back(variable_nz.second);

      // constraint index
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->push_back(ind_p);
}

void PiecewiseJerkProblem::set_x_bounds(    //定义l边界（核查维度）
    std::vector<std::pair<double, double>> x_bounds) {
  //CHECK_EQ(x_bounds.size(), num_of_knots_);
  x_bounds_ = std::move(x_bounds);  //使用 std::move() 将 x_bounds 移动到成员变量 x_bounds_ 中。
                                    //这意味着函数接受的 x_bounds 参数的所有权被转移给了 x_bounds_，从而避免了不必要的复制操作。
}

void PiecewiseJerkProblem::set_dx_bounds(   //定义dl边界（核查维度）
    std::vector<std::pair<double, double>> dx_bounds) {
  //CHECK_EQ(dx_bounds.size(), num_of_knots_);
  dx_bounds_ = std::move(dx_bounds);
}

void PiecewiseJerkProblem::set_ddx_bounds(   //定义ddl边界（核查维度）
    std::vector<std::pair<double, double>> ddx_bounds) {
  //CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  ddx_bounds_ = std::move(ddx_bounds);
}

/* void PiecewiseJerkProblem::set_dddx_bound(   //定义ddl边界（核查维度）
    std::vector<std::pair<double, double>> dddx_bound) {
  //CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  dddx_bound_ = std::move(dddx_bound);
} */

/* void PiecewiseJerkProblem::set_x_bounds(const double x_lower_bound,   //将l边界上下界赋值
                                        const double x_upper_bound) {
  for (auto& x : x_bounds_) {
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }
}

void PiecewiseJerkProblem::set_dx_bounds(const double dx_lower_bound,  //将dl边界上下界赋值
                                         const double dx_upper_bound) {
  for (auto& x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }
}

void PiecewiseJerkProblem::set_ddx_bounds(const double ddx_lower_bound,   //将dl边界上下界赋值
                                          const double ddx_upper_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }
} */

void PiecewiseJerkProblem::set_x_ref(const double weight_x_ref,    //给定参考线和参考线权重
                                     std::vector<double> x_ref) {
  //CHECK_EQ(x_ref.size(), num_of_knots_);
  weight_x_ref_ = weight_x_ref;
  // set uniform weighting
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, weight_x_ref);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

/* void PiecewiseJerkProblem::set_x_ref(std::vector<double> weight_x_ref_vec,  //给定参考线和分段x参考权重
                                     std::vector<double> x_ref) {
  //CHECK_EQ(x_ref.size(), num_of_knots_);
  //CHECK_EQ(weight_x_ref_vec.size(), num_of_knots_);
  // set piecewise weighting
  weight_x_ref_vec_ = std::move(weight_x_ref_vec);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
} */

void PiecewiseJerkProblem::set_end_state_ref(     //给定参考线终点和权重
    const std::array<double, 3>& weight_end_state,
    const std::array<double, 3>& end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ref_ = end_state_ref;
  has_end_state_ref_ = true;
}

void PiecewiseJerkProblem::FreeData(OSQPData* data) {   //释放 OSQPData 结构体中动态分配的内存。
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}



