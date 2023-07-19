# QP_Smooth_refrenceline
根据bilibili忠厚老实的老王的二次规划平滑参考线建模，使用c++语言调用OSQP库复现
使用matplotlib完成过程的可视化
## 依赖
--OSQP
求解线性规划的c++库
```shell
安装参考：https://blog.csdn.net/qjj18776858511/article/details/125963379
```
--matplotlib-cpp
    这个是基础的画图库，是很基本很重要经常会用到的画图库，安装也较为简单
```shell
pip3 install numpy
pip3 install matplotlib
git clone https://github.com/lava/matplotlib-cpp
CMakeLists.txt里面需要添加python相关的依赖，具体参考我的CMakeLists.txt
```
## 修改
QP.cpp中第7和20行修改待平滑点global_path_x和global_path_y的txt文档路径
## 编译运行
```shell
git clone https://github.com/socketnannannan/QP_Smooth_refrenceline.git
cd QP_Smooth_refrenceline
mkdir build && cd build
cmake .. && make
./osqp
```
## 效果
说明：算法原理及如何调试请参考pdf文件
![image]()
