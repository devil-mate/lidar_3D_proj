#ifndef _PID_CONTROLLER_INTERFACE_H
#define _PID_CONTROLLER_INTERFACE_H

#include <string.h>
typedef struct{
        float Kp ;
        float Ki ;
        float Kd ;
        float IntegralMax;
        float OutMax;
        float OutMin;
}PID_Params;

class PIDControllerInterface{
public:
    PIDControllerInterface();
    PIDControllerInterface(float kp,float ki,float kd);
    virtual ~PIDControllerInterface(){};
    // float update( float targetPoint, flaot currentValue, dt);
    virtual float calculate( float target, float currentValue,float dt)=0;
    PID_Params params_; //
private:

};


#endif

