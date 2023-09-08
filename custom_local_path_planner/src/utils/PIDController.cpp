
#include "PIDController.h"
PIDController::PIDController():PIDControllerInterface(),integral_(0),prevError_(0){
    
}
PIDController::PIDController(float kp,float ki,float kd):PIDControllerInterface(kp,ki,kd),integral_(0),prevError_(0){
    
}


float PIDController::calculate( float target, float currentValue, float dt) {
        float error = target - currentValue;
        integral_ += error * dt;
        float derivative = (error - prevError_) / dt;

        float output = params_.Kp * error + params_.Ki * integral_ + params_.Kd * derivative;
        prevError_ = error;
        // TODO 限制
        if (output > params_.OutMax) output = params_.OutMax;  
        else if (output < params_.OutMin) output = params_.OutMin;  
        return output;
}