#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include <limits>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "math.h"
using namespace std;
namespace plt = matplotlibcpp;

class MPC{
public:
//MPC状态设置
    int Np = 25;// predictive horizon 
    int Nc = 10;// control horizon
    int Nx = 5; //number of state variables
    int Nu = 1; //number of control inputs
    int Ny = 4; //number of output variables  
    double Ts = 0.05; //Set the sample time  采样周期
    Eigen::MatrixXd Q(Ny, Ny);  // cost weight factor 
    Q << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    double R  = 1; // cost weight factor 
    double S  = 1; // cost weight factor 
//车辆状态输入
    double RelativeDistance;   //两车实际车间距
    double EgoV            ;   //车辆纵向速度，单位：km/h-->m/s
    double RelativeV       ;   //车辆相对速度，单位：km/h-->m/s
    double EgoAcc          ;   //车辆纵向加速度，单位：g's-->m/s2 
    double EgoJerk         ;   //车辆加加速度，
//约束
    double vmin     =  0.0
    double vmax     =  200.0
    double amin     = -3.0;   // the min of deceleration   
    double amax     =  3.0;   // the max of acceleration
    double jmin     = -5.0;   // minimum limits of jerk
    double jmax     =  5.0;   // maximum limits of jerk
    double t        =  0.1;   //控制时间常数
    double dc       =  6.0;    //最小车间距
    Eigen::VectorXd M(4);
    Eigen::VectorXd N(4);
    M <<     dc ,  vmin  ,  amin  ,   jmin ; 
    N <<   10000 ,  vmax  ,  amax  ,  jmax ; 
//安全时距                                     
    double Timegap =1.4;

//状态矩阵
//x(k+1) = A x(k) + B u(k) + G w(k)
    Eigen::MatrixXd StateSpaceModel_A(Nx,Nx);
    StateSpaceModel_A <<   1 ,  0  ,  Ts , 1/2*Ts*Ts , 0,
                           0 ,  1  ,  0  ,   Ts      , 0,
                           0 ,  0  ,  1  ,   -Ts     , 0,
                           0 ,  0  ,  0  ,   1-Ts/t  , 1,
                           0 ,  0  ,  0  ,   -1/t    , 0;

    Eigen::VectorXd StateSpaceModel_B(Nx);
    StateSpaceModel_B <<   0 ,  0  ,  0  ,   Ts/t    , 1/t; 

//y(k) = C x(k) - Z
    Eigen::MatrixXd StateSpaceModel_C(Ny,Nx);
    StateSpaceModel_C <<   1   ,   -Timegap ,  0   ,   0   ,   0,
                           0   ,       0    ,  1   ,   0   ,   0,
                           0   ,       0    ,  0   ,   1   ,   0,
                           0   ,       0    ,  0   ,   0   ,   1;

    Eigen::VectorXd StateSpaceModel_Z(Ny);
    StateSpaceModel_Z << 10 ; 0 ; 0 ; 0;

//M_ <= L_ X^(k+p) <= N_
    Eigen::MatrixXd StateSpaceModel_L(Ny,Nx);
    StateSpaceModel_L <<  1 , 0 , 0 , 0 , 0
                          0 , 1 , 0 , 0 , 0
                          0 , 0 , 0 , 1 , 0
                          0 , 0 , 0 , 0 , 1;

//引入指数衰减函数 y_ref(k+i)=phi^i * y(k)
    Eigen::MatrixXd StateSpaceModel_phi(Ny, Ny); 
    StateSpaceModel_phi << 1, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 1;

//车辆状态输入
    Eigen::VectorXd xk(Nx);
    Eigen::VectorXd get_value(double RelativeDistance, double EgoV, double RelativeV, double EgoAcc, double EgoJerk);

//矩阵维度拓展
    func_Update

};