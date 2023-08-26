/********************************************************************************
 * class        PID controller with limit and anti-windup                       *
 *                                                                              *
 * file        Pid.h                                                            *
 * author      Ilya Galkin                                                      *
 * date        03.06.2020                                                       *
 * copyright   The MIT License (MIT)                                            *
 *                                                                              *
 ********************************************************************************/

/********************************************************************************
 * Include
 ********************************************************************************/

#include <math.h>

/********************************************************************************
 * Class PID controller
 ********************************************************************************/
/*
extern float error;
extern float integralFilter;
extern float derivativeFilter;
extern float proportionalComponent;
extern float integralComponent;
extern float derivativeComponent;
extern float tempPID;
extern float outputPID;
*/

typedef struct PidInputData
{
    float feedback;
    float reference;
    float deltaTimeSampling;
    float error;
    float integralFilter ;
    float derivativeFilter;
    float proportionalComponent;
    float integralComponent;
    float derivativeComponent;
    float tempPID;
    float outputPID;

    struct
    {
        float proportional;
        float integral;
        float derivative;
        float coefficientBackSaturation;
        float filterDerivative;
    }coefficient_t;

    struct
    {
        float lowThershold;
        float highThershold;
    }saturation_t ;
}PidInputData_t;

float PidCompute(PidInputData_t data);
float PidComputeAngle(PidInputData_t data);


