#include"../include/kalmanfilter.h"


kalmanfilter::kalmanfilter(float Q_angle, float Q_gyroBias , float R_measure):mfQ_angle(Q_angle),mfQ_gyroBias(Q_gyroBias),mfR_measure(R_measure)
{
    mmHT << 1,
            0;
}

void kalmanfilter::Initialize(Eigen::Vector2f Xposterior, Eigen::Matrix2f Pposterior) 
{
    mmXposterior = Xposterior;
    mmPposterior = Pposterior;
}

float kalmanfilter::GetAngle(float NewRate,float NewAngle)
{
    //预测
    mmFk << 1 , mfDeltaTime ,
            0 , 1;
    
    mmXpredictedLast << mfAngle , 
                        mfBias;

    mmB << mfDeltaTime , 
            0;     

    mmXpredictedCurrent = mmFk*mmXposterior+mmB*NewRate;
    
    //计算预测P
    mmQk << mfQ_angle , 0,
            0 , mfQ_gyroBias;

    mmPpredictedCurrent = mmFk*mmPposterior*mmFk.transpose()+mmQk*mfDeltaTime;

    //计算残差
    mfY = NewAngle - mmHT.transpose()*mmXpredictedCurrent;

    //计算Sk
    mfS = mmHT.transpose()*mmPpredictedCurrent*mmHT + mfR_measure; 

    //计算卡尔曼增益
    mmK = (mmPpredictedCurrent*mmHT)/mfS;
    
    //计算后验
    mmXposterior = mmXpredictedCurrent + mmK*mfY;

    //更新
    Eigen::Matrix2f I = Eigen::Matrix2f::Identity(); 
    mmPposterior = (I - mmK*mmHT.transpose())*mmPpredictedCurrent;
    
    //
    mfAngle = mmXposterior(0,0);
    return mfAngle;
}