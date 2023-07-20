#include <random>


struct ImuData
{
    double angle;
    double rate;
    /* data */
};

double generateGaussianNoise(double mean, double stddev);

ImuData simulateImuData(double trueRate, double rateBias, double rateBiasNoiseStddev, double rateNoiseStddev, double angleNoiseStddev, double dt);