#include "mpc.h"


//车辆状态输入
Eigen::VectorXd MPC::get_value(double RelativeDistance, double EgoV, double RelativeV, double EgoAcc, double EgoJerk){
    MPC.xk(0) = RelativeDistance;       //两车实际车间距
    MPC.xk(1) = EgoV;                   //车辆纵向速度，单位：km/h-->m/s
    MPC.xk(2) = RelativeV ;             //车辆相对速度，单位：km/h-->m/s
    MPC.xk(3) = EgoAcc;                 //车辆纵向加速度，单位：g's-->m/s2 
    MPC.xk(4) = EgoJerk;                //车辆加加速度，
    return xk;
}

