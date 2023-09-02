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

#include <pid.h>
#include "motor_ctrl.h"
/********************************************************************************
 * Class PID controller
 ********************************************************************************/

float PidCompute(PidInputData_t input_data)
{
    
    input_data.error = input_data.reference - input_data.feedback;

    input_data.proportionalComponent = input_data.coefficient_t.proportional * input_data.error;

    input_data.integralComponent += input_data.deltaTimeSampling * input_data.integralFilter;
    input_data.integralFilter = input_data.coefficient_t.integral * input_data.error + input_data.coefficient_t.coefficientBackSaturation * (input_data.outputPID - input_data.tempPID);

    input_data.derivativeFilter += input_data.deltaTimeSampling * input_data.derivativeComponent;
    input_data.derivativeComponent = (input_data.coefficient_t.derivative * input_data.error - input_data.derivativeFilter) * input_data.coefficient_t.filterDerivative;

    input_data.outputPID = input_data.tempPID = input_data.proportionalComponent + input_data.integralComponent + input_data.derivativeComponent;
    input_data.outputPID = fmin(fmax(input_data.outputPID, input_data.saturation_t.lowThershold), input_data.saturation_t.highThershold);

    return input_data.outputPID;
}

float PidComputeAngle(PidInputData_t input_data)
{
    
    input_data.error = input_data.reference - input_data.feedback;
    if(input_data.error>PI)
    {
        input_data.error-=2*PI;
    }
    if(input_data.error<-PI)
    {
        input_data.error+=2*PI;
    }
    input_data.proportionalComponent = input_data.coefficient_t.proportional * input_data.error;

    input_data.integralComponent += input_data.deltaTimeSampling * input_data.integralFilter;
    input_data.integralFilter = input_data.coefficient_t.integral * input_data.error + input_data.coefficient_t.coefficientBackSaturation * (input_data.outputPID - input_data.tempPID);

    input_data.derivativeFilter += input_data.deltaTimeSampling * input_data.derivativeComponent;
    input_data.derivativeComponent = (input_data.coefficient_t.derivative * input_data.error - input_data.derivativeFilter) * input_data.coefficient_t.filterDerivative;

    input_data.outputPID = input_data.tempPID = input_data.proportionalComponent + input_data.integralComponent + input_data.derivativeComponent;
    input_data.outputPID = fmin(fmax(input_data.outputPID, input_data.saturation_t.lowThershold), input_data.saturation_t.highThershold);

    return input_data.outputPID;
}



