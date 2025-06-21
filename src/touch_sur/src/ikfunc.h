#ifndef __IKFUNC_H__
#define __IKFUNC_H__

#include <iostream>
#include <cmath>
#include <array>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <array>

// Function for inverse kinematics calculation
std::array<double, 2> inverseKinematics(double x, double y, double z) {
    // Calculate bending angle beta
    double beta = 2 * std::atan2(std::sqrt(x * x + y * y), z);

    // Calculate rotation angle phi
    double phi = std::atan2(y, x);
    if (phi < 0) {
        phi += 2 * M_PI;  // Adjust phi to be in the range [0, 2π]
    }

    // Return the results as an array
    return {beta, phi};
}

// Function for inverse kinematics calculation
void inverseKinematics(double L, double x, double y, double z, double &beta, double &phi) {
    // Calculate bending angle beta
    beta = 2 * std::atan2(std::sqrt(x * x + y * y), z);

    // Calculate rotation angle phi
    phi = std::atan2(y, x);
    if (phi < 0) {
        phi += 2 * M_PI;  // Adjust phi to be in the range [0, 2π]
    }
}

// Function to calculate rope length change from position
std::array<double, 3> calculateRopeLengthChangeFromPosition(double L, double r, double x, double y, double z) {
    // Inverse Kinematics - Calculate beta and phi
    double beta, phi;
    inverseKinematics(L, x, y, z, beta, phi);

    // Equivalent bending radius
    double R = L / beta;

    // Calculate length change for each of the 3 ropes
    std::array<double, 3> delta_l = {0.0, 0.0, 0.0};  // Store changes for 3 ropes
    for (int i = 0; i < 3; ++i) {
        // Bending radius for each rope
        double Ri = R - r * std::cos(phi - (2 * M_PI * i / 3));

        // Length change for this rope
        delta_l[i] = L - Ri * beta;
    }

    return delta_l;
}
//将delta_l写入文件，追加的形式
void writeDeltaLToFile(const std::string& filename, const std::array<double, 3>& delta_l) {
    // 打开文件用于追加写入
    std::ofstream out_file(filename, std::ios_base::app);
    
    // 检查文件是否成功打开
    if (!out_file) {
        std::cerr << "无法打开文件 " << filename << " 进行写入！" << std::endl;
        return;
    }

    
    // 写入 delta_l 到文件
    for (const auto& value : delta_l) {
        out_file << value <<" ";
    }
    out_file << std::endl;

    // 关闭文件
    out_file.close();
    std::cout << "文件 " << filename << " 写入成功！" << std::endl;
}

template <size_t N>
void writeArrayToFile(const std::string& filename, const std::array<double, N>& data) {
    // std::ofstream file(filename);
    std::ofstream file(filename, std::ios_base::app); // 以追加模式打开文件
    if (file.is_open()) {
        for (const auto& value : data) {
            file << value << " ";
        }
        file << std::endl; // 每次写入后换行
        file.close();
        std::cout << "文件 " << filename << " 写入成功！" << std::endl;
    } else {
        std::cerr << "无法打开文件: " << filename << std::endl;
    }
}



//以圆上的点作为(x,y,z)路径点，求出逆解delta_l,并以追加的形式写入文件，初始值半径=5，圆心={0,0,5}，点的数量=100
void writeCirclePointToFile(double radius = 10, const std::vector<double>& center = {0, 0, 5}, int num_points = 100)
{
    // double radius = 10;
    // const std::vector<double> &center = {0, 0, 5};
    // int num_points = 100; // 计算圆上的点的数量
    // z初始化为center[2] = 5
    std::vector<double> theta(num_points), x(num_points), y(num_points), z(num_points, center[2]);
    std::array<double, 3> delta_l;
    // 计算圆上的点
    for (int i = 0; i < num_points; ++i)
    {
        theta[i] = 2 * M_PI * i / num_points;
        x[i] = center[0] + radius * cos(theta[i]);
        y[i] = center[1] + radius * sin(theta[i]);

        delta_l = calculateRopeLengthChangeFromPosition(10, 3, x[i], y[i], 5);
        // 将delta_l写入文件
        writeDeltaLToFile("delta_l.txt", delta_l);
        //writeArrayToFile("delta_l.txt", delta_l);
        // ROS_INFO("delta_l[0] = %f,delta_l[1] = %f,delta_l[2] = %f\n", delta_l[0], delta_l[1], delta_l[2]);
    }
}





#endif  // __IKFUNC_H__