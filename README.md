# touch_ws
Master-slave control of 822 laboratory, using the PHANTOM Omni to control the aubo i5 robotic arm

## 前提条件
1. ROS 安装
2. 3D system touch 安装
参考：[3D Systems Touch在20.04 ubuntu（noetic）环境下的配置与使用](https://blog.csdn.net/weixin_52725622/article/details/134164760?fromshare=blogdetail&sharetype=blogdetail&sharerId=134164760&sharerefer=PC&sharesource=dukangyou&sharefrom=from_link)


## 运行
依次执行：
1. 克隆仓库
```bash
git clone git@github.com:dkyou/touch_ws.git
```
2. 编译
```bash
cd touch_ws
catkin_make
```
3. 运行phantom omni touch
```
roslaunch omni_common omni_state.launch
```
启动rviz，并且显示phantom omni touch 即为成功

4. 运行 aubo i5
```bash
roslaunch aubo_i5_moveit_config demo.launch
```
启动rviz，并且显示aubo i5 即为成功
5. 计算逆解
```bash
roslaunch aubo_tele aubo_tele.launch
```
6. 效果
可以在rivz仿真环境中，使用phantom omni touch控制aubo i5进行主从控制
参考：
1. fork仓库：https://github.com/fsuarez6/phantom_omni
2. 来碗麻酱的主页：[来碗麻酱](https://blog.csdn.net/weixin_52725622?type=blog)
