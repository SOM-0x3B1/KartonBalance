#include "control.h"
#include "comm.h"
#include "imu.h"
#include "motor.h"
#include "math.h"
#include "encoder.h"


const float g = 9.8067;


volatile float targetSpeed = 0;
volatile float targetPitchAngle = 0; // keep this angle to achieve balance

volatile float Kp = 500;          // (P)roportional Tuning Parameter
volatile float Ki = 600;          // (I)ntegral Tuning Parameter        
volatile float Kd = 160;          // (D)erivative Tuning Parameter       
volatile float maxPID = 1024;     // the maximum value that can be output

volatile float tau = 0.002;

volatile float lastPitch = 0;     // the last sensor value
volatile float lastError = 0;     // the last error value
volatile int lastDecTargetAngle = 0;

volatile float iTerm = 0;         // used to accumulate error (integral)


/// Calculate optimal motor speed
void controlPitch(){
    // Calculate error between target and current values
	volatile float error = (targetPitchAngle /*+ antiDrift_offsetAngle*/) - compPitch;
	iTerm += error * IMU_EVAL_INTERVAL;

	// Calculate the derivative term
	volatile float dTerm = (error - lastError) / IMU_EVAL_INTERVAL / 10;
    
	lastError = error;

	// Multiply each term by its constant, and add it all up
	volatile float result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);    
    //result = fabsf(result);

	// Limit PID value to maximum values
	if (result > maxPID) 
        result = maxPID;
    else if (result < -maxPID)
        result = -maxPID;
    
    motor_evalDirection(result);
    
    if(sendPID){
        outP = error * Kp * 10;
        outI = iTerm * Ki * 10;
        outD = dTerm * Kd * 10;
        outPID = result * 10;        
    }
    
    result = fabsf(result);    
    motor_LPercentage = result / maxPID;
    motor_RPercentage = result / maxPID;
}


float controlLinearVelocity_getA(){
    float dv = targetSpeed - (encoderL.speed + encoderR.speed) / 2;
    return dv / tau;
}

void controlLinearAcc(){
    targetPitchAngle = atanf(controlLinearVelocity_getA() / g);
    if(targetPitchAngle > 5)
        targetPitchAngle = 5;
    else if (targetPitchAngle < -5)
        targetPitchAngle = -5;
}