#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

/***
 * 普通PID/位置PID
*/
#include "PIDControllerInterface.h"



class PIDController: public PIDControllerInterface{
public:
    PIDController();
    PIDController(float kp,float ki,float kd);
    virtual ~PIDController(){};
    virtual float calculate( float target, float currentValue,float dt) override;
private:
    float integral_; //普通pid积分累加
    float prevError_; //前一时刻误差
};


#endif

