#include "../include/kalmanfilter.h"
#include "../include/SimulateImuData.h"
#include "../thirdparty/matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main()
{
    kalmanfilter kalmanfilter(0.001, 0.03, 0.03);
    // initialize kalmanfilter
    Eigen::Vector2f X;
    Eigen::Matrix2f P;
    X << 1,
        0;
    P << 0, 0,
        0, 0;
    kalmanfilter.Initialize(X, P);
    double dt = 0.01;                  // 时间步长
    double rateBias = 0.8;             // 角速度偏移量
    kalmanfilter.mfDeltaTime = 0.01;
    double rateBiasNoiseStddev = 0.07; // 角速度偏移量的噪声标准差
    ImuData data = {0, 0};             // 初始的IMU数据
    std::vector<double> imu_data;      // 存储IMU数据的向量
    std::vector<float> kalman_data;   // 存储卡尔曼滤波后的数据

    for (int i = 0; i < 1000; i++)
    {                                                                               // 模拟1000个时间步
        data = simulateImuData(1.0, rateBias, rateBiasNoiseStddev, 0.01, 0.01, dt); // 生成模拟的IMU数据
        imu_data.push_back(data.angle);                                             // 将IMU数据添加到向量中

        float kalman_angle = kalmanfilter.GetAngle(data.rate, data.angle); // 使用卡尔曼滤波器处理数据
        kalman_data.push_back(kalman_angle);                                // 将卡尔曼滤波后的数据添加到向量中

        // 绘制IMU数据和卡尔曼滤波后的数据
        plt::clf(); // 清除当前的图像
        plt::named_plot("IMU data", imu_data);
        plt::named_plot("Kalman filter", kalman_data);
        plt::legend();
        plt::pause(0.01); // 暂停0.01秒，让图像有时间更新
    }

    return 0;
}
