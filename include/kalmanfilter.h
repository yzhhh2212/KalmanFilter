#include <Eigen/Dense>

class kalmanfilter
{
public:
    kalmanfilter(float Q_angle, float Q_gyroBias , float R_measure);
public:
    void Initialize(Eigen::Vector2f Xposterior, Eigen::Matrix2f Pposterior);
    float GetAngle(float NewRate,float NewAngle);
public:
    float mfQ_angle;
    float mfQ_gyroBias;
    float mfR_measure;
    float mfAngle;
    float mfBias;
    float mfPk;
    float mfDeltaTime;
    float mfRate;
    float mfNewRate; 
    float mfAngleBias;
    float mfY;
    float mfS;
    Eigen::Vector2f mmK;
    Eigen::Vector2f mmHT;
    Eigen::Vector2f mmXpredictedCurrent;
    Eigen::Vector2f mmXpredictedLast;
    Eigen::Vector2f mmXposterior;
    Eigen::Vector2f mmB;
    Eigen::Matrix2f mmFk;
    Eigen::Matrix2f mmQk;
    Eigen::Vector2f mmHk;
    Eigen::Matrix2f mmPpredictedLast;
    Eigen::Matrix2f mmPpredictedCurrent;
    Eigen::Matrix2f mmPposterior;
};