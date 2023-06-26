# LocalPlanner
1.随机动态障碍物生成的可视化双车道场景  
2.基于MPC的局部轨迹规划算法  
3.matplotlib可视化场景
## 依赖
--osqp-eigen
求解二次规划的c++库
```shell
./install_osqp.sh
```
--matplotlib-cpp
    这个是基础的画图库，是很基本很重要经常会用到的画图库，安装也较为简单
```shell
pip3 install numpy
pip3 install matplotlib
git clone https://github.com/lava/matplotlib-cpp
复制matplotlibcpp.h到/usr/include文件夹中，一劳永逸
CMakeLists.txt里面需要添加python相关的依赖，具体参考我的CMakeLists.txt
```
## 编译运行
```shell
git clone https://github.com/dongtaihong/S.git

```
## 效果
说明：算法原理及如何调试请参考pdf文件