#pragma once
#include "matplotlibcpp.h"
#include <tuple>
#include <utility>
#include <vector>
#include "osqp/osqp.h"
namespace plt = matplotlibcpp;
class PiecewiseJerkProblem {
 public:
  PiecewiseJerkProblem(const size_t num_of_knots, const double delta_s,  //构造函数初始化
                       const std::array<double, 3>& x_init);

  ~PiecewiseJerkProblem() = default;    //析构函数

    //边界函数声明
  void set_x_bounds(std::vector<std::pair<double, double>> x_bounds);

  //void set_x_bounds(const double x_lower_bound, const double x_upper_bound);

  void set_dx_bounds(std::vector<std::pair<double, double>> dx_bounds);

  //void set_dx_bounds(const double dx_lower_bound, const double dx_upper_bound);

  void set_ddx_bounds(std::vector<std::pair<double, double>> ddx_bounds);

  //void set_ddx_bounds(const double ddx_lower_bound, const double ddx_upper_bound);

  //void set_dddx_bound(std::vector<std::pair<double, double>> dddx_bound);

  void set_x_ref(const double weight_x_ref, std::vector<double> x_ref);  //与参考线偏差的均匀加权

  //void set_x_ref(std::vector<double> weight_x_ref_vec, std::vector<double> x_ref);    //分段x参考权重？？

  void set_end_state_ref(const std::array<double, 3>& weight_end_state,    //停车点权重
                         const std::array<double, 3>& end_state_ref);  

  bool Optimize(const int max_iter = 4000); 
  int max_iter = 4000;        
/*   void set_dddx_bound(const double dddx_bound) {           //？？
    set_dddx_bound(-dddx_bound, dddx_bound);
  } */

/*   void set_dddx_bound(const double dddx_lower_bound,       //？？
                      const double dddx_upper_bound) {
    dddx_bound_.first = dddx_lower_bound;
    dddx_bound_.second = dddx_upper_bound;
  } */

    //权重声明
/*   void set_weight_x(const double weight_x) { weight_x_ = weight_x; }   //0阶导权重

  void set_weight_dx(const double weight_dx) { weight_dx_ = weight_dx; }  //1阶导权重

  void set_weight_ddx(const double weight_ddx) { weight_ddx_ = weight_ddx; }  //2阶导权重

  void set_weight_dddx(const double weight_dddx) { weight_dddx_ = weight_dddx; }   //3阶导权重 

  void set_scale_factor(const std::array<double, 3>& scale_factor) {    //设置比例因子
    scale_factor_ = scale_factor;
  }

  void set_x_ref(const double weight_x_ref, std::vector<double> x_ref);  //与参考线偏差的均匀加权

  void set_x_ref(std::vector<double> weight_x_ref_vec,    //分段x参考权重？？
                 std::vector<double> x_ref);

  void set_end_state_ref(const std::array<double, 3>& weight_end_state,    //停车点权重
                         const std::array<double, 3>& end_state_ref);

  virtual bool Optimize(const int max_iter = 4000);    //？？

  const std::vector<double>& opt_x() const { return x_; }  //？？

  const std::vector<double>& opt_dx() const { return dx_; }   //？？

  const std::vector<double>& opt_ddx() const { return ddx_; }   //？？
*/
// protected:
 public:
  // naming convention follows osqp solver.
  void CalculateKernel(std::vector<c_float>* P_data,      //计算 Hessian 矩阵
                               std::vector<c_int>* P_indices,
                               std::vector<c_int>* P_indptr);

  void CalculateOffset(std::vector<c_float>* q);     //计算g矩阵

  void CalculateAffineConstraint(std::vector<c_float>* A_data,    //计算A矩阵
                                         std::vector<c_int>* A_indices,
                                         std::vector<c_int>* A_indptr,
                                         std::vector<c_float>* lower_bounds,
                                         std::vector<c_float>* upper_bounds);

  virtual OSQPSettings* SolverDefaultSettings();   //？？

  OSQPData* FormulateProblem();    // 函数的声明返回一个指向 OSQPData 类型对象的指针。

  void FreeData(OSQPData* data);   //释放 OSQPData 结构体中动态分配的内存。

  template <typename T>   //模版？？
  T* CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

// protected:
 public:
  size_t num_of_knots_;  //待平滑的点数

  // output
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;

  std::array<double, 3> x_init_;  //初始点状态
  std::array<double, 3> scale_factor_ = {1.0, 1.0, 1.0};   //比例因子

  std::vector<std::pair<double, double>> x_bounds_;   //边界值
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  std::pair<double, double> dddx_bound_;

  double weight_x_ = 1.0;    //各阶导权重
  double weight_dx_ = 1.0;
  double weight_ddx_ = 1.0;
  double weight_dddx_ = 1.0;

  double delta_s_ = 1.0;   //间隔时间

  bool has_x_ref_ = false;   //有无参考线
  double weight_x_ref_ = 1.0;  //参考线权重
  std::vector<double> x_ref_;  //参考线l

  // un-uniformed weighting 不统一的权重
  std::vector<double> weight_x_ref_vec_;            //分段x参考权重？？

  bool has_end_state_ref_ = false;      //有无终点状态参考
  std::array<double, 3> weight_end_state_ = {2.0, 2.0, 2.0};   //终点权重
  std::array<double, 3> end_state_ref_;     //终点参考线偏差
};

