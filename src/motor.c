#include "motor.h"

#include "project.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "control.h"
#include "imu.h"
#include "comm.h"



const int motor_fullThrottle = MOTOR_PWM_MAX - MOTOR_PWM_MIN;


const MotorIn motor_input_sleep = (MotorIn) {0, 0};
const MotorIn motor_input_forward = (MotorIn) {0, 1};
const MotorIn motor_input_backwards = (MotorIn) {1, 0};
const MotorIn motor_input_brake = (MotorIn) {1, 1};

volatile MotorIn motor_curr_input; // current motor input

volatile MotorState motor_sate = MOTOR_SLEEP;
volatile bool motor_firstStart = true; // the motor hasn't been moving before
volatile Direction motor_newDirection;
volatile Direction motor_lastDirection;
volatile float motor_LPercentage = 0, motor_RPercentage = 0; // PWM duty cicle percentage



volatile bool encoderEvalReady = false;


/// Get direction according to the current pitch
void motor_evalDirection(float PIDres){    
    if(PIDres < 0 && PIDres < 0){ // tilting forwars
        motor_newDirection = FORWARD;
        motor_curr_input = motor_input_forward;
    }
    else { // tilting backwards
        motor_newDirection = BACWARDS;
        motor_curr_input = motor_input_backwards;
    }
    
    if(motor_firstStart)
        motor_lastDirection = motor_newDirection;  // skip direction change check & save current
    
    if( motor_newDirection != motor_lastDirection) // direction change check
        motor_sate = MOTOR_BRAKE;
    
    motor_lastDirection = motor_newDirection;
}

/// Set direction on the motor controller
void motor_setDirection(){
    Pin_Motor_In_1_Write(motor_curr_input.first);
    Pin_Motor_In_2_Write(motor_curr_input.second);
    Pin_Motor_In_3_Write(motor_curr_input.first);
    Pin_Motor_In_4_Write(motor_curr_input.second);
}


/// Set motor speed to 0
void motor_resetPWM(){
    motor_LPercentage = 0;
    motor_RPercentage = 0;
}

/// Set motor speed to the calculated PWM
void motor_setPWM(){
    int lPWM =(int)(motor_fullThrottle * motor_LPercentage);
    int rPWM =(int)(motor_fullThrottle * motor_RPercentage);
    PWM_Motor_WriteCompare1(MOTOR_PWM_MIN + lPWM);
    PWM_Motor_WriteCompare2(MOTOR_PWM_MIN + rPWM);
}