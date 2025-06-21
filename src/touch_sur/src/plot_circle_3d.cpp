#include <iostream>
#include <vector>
#include <cmath>


#include <fstream>
#include <array>
#include <algorithm>
#include <iterator>
#include "ikfunc.h"

using namespace std;
void plot_circle_3d(double radius, const std::vector<double>& center) {
    // 定义圆的参数范围
    int num_points = 100;  // 计算圆上的点的数量
    std::vector<double> theta(num_points), x(num_points), y(num_points), z(num_points, center[2]);
    std::array<double, 2> beta_phi = {0.0, 0.0};
    std::array<double, 3> track = {0.0, 0.0,1.0};
    // 计算圆上的点
    for (int i = 0; i < num_points; ++i) {
        theta[i] = 2 * M_PI * i / num_points;
        x[i] = center[0] + radius * cos(theta[i]);track[0] = x[i];
        y[i] = center[1] + radius * sin(theta[i]);track[1] = y[i];
        
        // 将x,y,z写入文件
        writeArrayToFile("track.txt", track);
        beta_phi = inverseKinematics(x[i], y[i], 1.0);
        // 将beta和phi写入文件
        writeArrayToFile("beta_phi.txt", beta_phi);
    }

    
}

int main() {
    // 定义圆的半径和圆心位置
    double radius = 5.0;
    std::vector<double> center = {0, 0, 1.0};
    
    // 绘制圆
    plot_circle_3d(radius, center);

    return 0;
}
