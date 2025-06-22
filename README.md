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

## vscode 配置
1. 默认使用catkin_make 编译
创建`.git/tasks.json`,填入以下内容
```json
{
    // 有关 tasks.json 格式的文档，请参见
        // https://go.microsoft.com/fwlink/?LinkId=733558
        "version": "2.0.0",
        "tasks": [
            {
                "label": "catkin_make:debug", //代表提示的描述性信息
                "type": "shell",  //可以选择shell或者process,如果是shell代码是在shell里面运行一个命令，如果是process代表作为一个进程来运行
                "command": "catkin_make",//这个是我们需要运行的命令
                "args": [],//如果需要在命令后面加一些后缀，可以写在这里，比如-DCATKIN_WHITELIST_PACKAGES=“pac1;pac2”
                "group": {"kind":"build","isDefault":true},
                "presentation": {
                    "reveal": "always"//可选always或者silence，代表是否输出信息
                },
                "problemMatcher": "$msCompile"
            }
        ]
    }
```
之后只需要编译ros项目`Ctrl+Shift+B`即可

2. vscode头文件搜索
创建：`c_cpp_properties.json`文件,填入以下内容
```json
{
  "configurations": [
    {
      "name": "linux-gcc-x64",
      "includePath": [
        "${workspaceFolder}/**",
        "/opt/ros/noetic/include"//添加实际头文件搜索路径
      ],
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "${default}",
      "cppStandard": "${default}",
      "intelliSenseMode": "linux-gcc-x64",
      "compilerArgs": [
        ""
      ]
    }
  ],
  "version": 4
}
之后就vscode就可以跳转函数和变量了
```


## 参考：
1. fork仓库：https://github.com/fsuarez6/phantom_omni
2. 来碗麻酱的主页：[来碗麻酱](https://blog.csdn.net/weixin_52725622?type=blog)
