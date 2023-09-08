
#include "PIDControllerInterface.h"

PIDControllerInterface::PIDControllerInterface(){
     memset(&params_,0,sizeof(params_));
}
PIDControllerInterface::PIDControllerInterface(float kp,float ki,float kd){
    memset(&params_,0,sizeof(params_));
    params_.OutMax=99999.0;
    params_.Kp=kp;
    params_.Kd=kd;
    params_.Ki=ki;

}
