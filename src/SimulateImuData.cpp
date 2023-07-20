#include "../include/SimulateImuData.h"


//struct ImuData
//{
    //double angle;
    //double rate;
    ///* data */
//};

double generateGaussianNoise(double mean, double stddev) {
    static std::random_device rd;
    static std::mt19937 generator(rd());
    std::normal_distribution<double> distribution(mean, stddev);

    return distribution(generator);
}

// 模拟IMU数据
ImuData simulateImuData(double trueRate, double rateBias, double rateBiasNoiseStddev, double rateNoiseStddev, double angleNoiseStddev, double dt) {
    ImuData data;
    data.rate = trueRate + generateGaussianNoise(0, rateNoiseStddev);  // 角速度加上高斯噪声
    double noisyRateBias = rateBias + generateGaussianNoise(0, rateBiasNoiseStddev);  // 带有噪声的角速度偏移量
    data.angle += (data.rate - noisyRateBias) * dt + generateGaussianNoise(0, angleNoiseStddev);  // 角度是（角速度减去带有噪声的角速度偏移量）乘以时间，再加上高斯噪声

    return data;
}
