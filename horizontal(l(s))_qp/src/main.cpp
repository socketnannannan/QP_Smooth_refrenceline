#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <limits>
#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include "QP.h"
using namespace std;

int main(){
    size_t num_of_knots = 4000;
    PiecewiseJerkProblem path_QP(num_of_knots, 1, {0.0, 0.0, 0.0});  //初始化（点数， 时间间隔， 初始状态）

//l边界
    std::vector<std::pair<double, double>> x_bounds;
    for (int i = 0; i < num_of_knots; ++i) {
        x_bounds.push_back(std::make_pair(-8.0, 8.0));
    }
    path_QP.set_x_bounds(x_bounds);    //赋值l边界（核查维度）   
//dl边界
    std::vector<std::pair<double, double>> dx_bounds;
    for (int i = 0; i < num_of_knots; ++i) {
        dx_bounds.push_back(std::make_pair(-2.0, 2.0));
    }
    path_QP.set_dx_bounds(dx_bounds);   //赋值dl边界（核查维度）
//ddl边界
    std::vector<std::pair<double, double>> ddx_bounds;
    for (int i = 0; i < num_of_knots; ++i) {
        ddx_bounds.push_back(std::make_pair(-1.0, 1.0));
    }
    path_QP.set_ddx_bounds(ddx_bounds);    //赋值ddl边界（核查维度）

//dddl边界
    std::pair<double, double> dddx_bound_ = path_QP.dddx_bound_;
    dddx_bound_ = std::make_pair(-0.1, 0.1);

/* //dddl边界
    std::vector<std::pair<double, double>> dddx_bound;
    for (int i = 0; i < num_of_knots; ++i) {
        dddx_bound.push_back(std::make_pair(-0.1, 0.1));
    }
    path_QP.set_dddx_bound(dddx_bound);    //赋值dddl边界（核查维度） */

/*     void PiecewiseJerkProblem::set_x_bounds(const double x_lower_bound, const double x_upper_bound)   //将l边界上下界赋值                                        
    void PiecewiseJerkProblem::set_dx_bounds(const double dx_lower_bound, const double dx_upper_bound)  //将dl边界上下界赋值
    void PiecewiseJerkProblem::set_ddx_bounds(const double ddx_lower_bound, const double ddx_upper_bound)   //将dl边界上下界赋值 */

//参考线值
    std::vector<double> x_ref;
    for (int i = 0; i < num_of_knots; ++i) {
        x_ref.push_back(2);
    }                                          
    path_QP.set_x_ref(50, x_ref);    //给定参考线和参考线权重 

/*     std::vector<double> weight_x_ref_vec;
    for (int i = 0; i < num_of_knots; ++i) {
        weight_x_ref_vec.push_back(4);
    }
    path_QP.set_x_ref(weight_x_ref_vec, x_ref);  //给定参考线和分段x参考权重 */


    std::array<double, 3> end_state_ref = {-2.0, -1.0, -0.5};                             
    path_QP.set_end_state_ref({500.0, 500.0, 500.0}, end_state_ref); //给定参考线终点和权重

    bool success =  path_QP.Optimize(4000);                                
}