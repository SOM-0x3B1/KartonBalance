#include "project.h"
#include "stdio.h"
#include "mpu6050.h"
#include "math.h"

#include "control.h"
#include "comm.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"



volatile bool sleep = false; // suspend motors

/// Update the LED
void updateLED(){
    if(sleep)
        PWM_LED_WriteCompare(50);
    else
        PWM_LED_WriteCompare(800);
}



//=====[ Interrupts ]==================

/// get gyro sample
CY_ISR(GyroSampleIT){    
    MPU6050_getMotion6(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ);
    imu_sampleReady = true;
    Timer_GY87_Sample_STATUS;
}

/// Encoder interrupt count
CY_ISR(EncoderLeftIT){
    if(motor_newDirection == FORWARD)
        encoderL.currCount++;
    else
        encoderL.currCount--;
}
CY_ISR(EncoderRightIT){
    if(motor_newDirection == FORWARD && motor_sate == MOTOR_GO)
        encoderR.currCount++;
    else if (motor_sate == MOTOR_GO)
        encoderR.currCount--;
}
/// Evaluate encoder interrupt count
CY_ISR(EncoderEvalIT){    
    encoder_update(&encoderL);
    encoder_update(&encoderR);
    encoderEvalReady = true;
    Timer_Motor_Encoder_Eval_STATUS;
}


/// Button interrupt to toggle sleep mode
CY_ISR(ToggleSleepIT){
    sleep = !sleep;    
    motor_sate = sleep ? MOTOR_SLEEP : MOTOR_GO;
    updateLED();
    if(!sleep){
        encoder_clear(&encoderL);
        encoder_clear(&encoderR);
        iTerm = 0;
    }
}

/// Read UART data
CY_ISR(UARTEvalIT){    
    if(currentUARTSource == SOURCE_WAITING){
        if(UART_USB_GetRxBufferSize())
            currentUARTSource = SOURCE_USB;
        else if(UART_Bluetooth_GetRxBufferSize())
            currentUARTSource = SOURCE_BLUETOOTH;
        else
            currentUARTSource = SOURCE_WAITING;
    }
    Timer_UART_Eval_STATUS;
}

/// Ready to send UART data
CY_ISR(UARTSendIT){
    sendData = true;
    Timer_UART_Send_STATUS;
}




//=====[ Initialization ]==================

/// initialize things
void init() {        
    // init USB UART
    UART_USB_Start();    
    UART_USB_PutString("\n\rCOM Port Open\n\r");
    
    UART_Bluetooth_Start();
    UART_Bluetooth_PutString("\n\rCOM Port Open\n\r");
    
    // init IMU
    I2C_GY87_Start();
    MPU6050_init();
	MPU6050_initialize();
    MPU6050_setMasterClockSpeed(13); // 400 kbps
    MPU6050_setDLPFMode(1); // 184 Hz, supports 1 kHz sample rate
    
    sprintf(outBuf, MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    UART_dual_PutString(true);
    
    // calibrate IMU offsets
    sprintf(outBuf, "Calbirating...\n\r");
    UART_dual_PutString(true);
    imu_calibrate(500);
    sprintf(outBuf, "Calbiration done\n\n\r");
    UART_dual_PutString(true);
    
    // start timers
    Clock_Timer_G87_Sample_Start();
    Clock_Timer_Motor_Encoder_Eval_Start();
    Clock_Timer_UART_Start();
    Timer_GY87_Sample_Start(); 
    Timer_UART_Eval_Start();
    Timer_UART_Send_Start();
    Timer_Motor_Encoder_Eval_Start();
    
    sprintf(outBuf, "Timers started\n\r");
    UART_dual_PutString(true);
    
    // start motor control
    Clock_Motor_PWM_Start();
    PWM_Motor_Start();
    motor_curr_input = motor_input_sleep;
    
    encoder_init(&encoderL);
    encoder_init(&encoderR);
    
    sprintf(outBuf, "Motor control started\n\r");
    UART_dual_PutString(true);
    
    // start status LED 
    PWM_LED_Start();
    PWM_LED_BRIGHTNESS_Start();
    
    sprintf(outBuf, "LED blinking\n\r");
    UART_dual_PutString(true);
    
    // register interrupts
    isr_Sample_GY87_StartEx(GyroSampleIT);
    isr_Inc_Motor_Encoder_L_StartEx(EncoderLeftIT);
    isr_Inc_Motor_Encoder_R_StartEx(EncoderRightIT);   
    isr_Eval_Motor_Encoder_StartEx(EncoderEvalIT);
    isr_Toggle_Sleep_StartEx(ToggleSleepIT);
    isr_UART_Eval_StartEx(UARTEvalIT);
    isr_UART_Send_StartEx(UARTSendIT);
    
    sprintf(outBuf, "Interrupts registered\nStarting...\n\n\r");
    UART_dual_PutString(true);
}



int main(void) {   
    init();   
    
    CyGlobalIntEnable; /* Enable global interrupts. */
 
        
    for(;;) {              
        
        UART_enum();
        
        if(imu_sampleReady) {
            // apply offstets and accumulate samples
            AX += ((float)CAX-AXoff);
            AY += ((float)CAY-AYoff);
            AZ += ((float)CAZ-AZoff);
            
            GX += ((float)CGX-GXoff);
            GY += ((float)CGY-GYoff);
            GZ += ((float)CGZ-GZoff);
            
            sampleCount++;
            
            if(sampleCount >= IMU_SAMPLE_COUNT)
                imu_evalReady = true;
            
            imu_sampleReady = false;
        }
    
        if(imu_evalReady) {
            if(sampleCount > 0)
               imu_calcAngles(sampleCount);   
     
            // reset samples
            AX = 0; AY = 0; AZ = 0;
            GX = 0; GY = 0; GZ = 0;           
            sampleCount = 0;   
            
            gyroStarted = true;            
            
            if(!sleep){
                controlLinearAcc();
                controlPitch();                
            }
            
            imu_evalReady = false;
        }
        
        
        // ready to send UART output
        if(sendData){
            outBuf[0] = '\0';   
            int decCompPitch = compPitch*100;
            int decCompRoll = compRoll*100;
            int decTargetAngle = targetPitchAngle*100;
            if(sendGyro && (decCompPitch != lastDecCompPitch || decCompRoll != lastDecCompRoll || decTargetAngle != lastDecTargetAngle)){   
                sprintf(outBufAngles, "GA %d %d %d\n\r",  decCompPitch, decCompRoll, decTargetAngle);
                strcat(outBuf, outBufAngles);
                lastDecCompPitch = decCompPitch;
                lastDecCompRoll = decCompRoll;
            }
            
            int decLSpeed = encoderL.speed * 10000;
            int decRSpeed = encoderR.speed * 10000;
            if(sendMotor && (encoderL.speed != lastDecEncoderSpeedL || encoderR.speed != lastDecEncoderSpeedR)){
                sprintf(outBufSpeed, "MS %d %d\n\r", decLSpeed, decRSpeed); 
                strcat(outBuf, outBufSpeed);
                lastDecEncoderSpeedL = decLSpeed;
                lastDecEncoderSpeedR = decRSpeed;
            }
            
            if(sendPID && (outPID != lastOutPID || outP != lastOutP || outI != lastOutI || outD != lastOutD)){
                sprintf(outBufPID, "PR %d %d %d %d\n\r",  outP, outI, outD, outPID);
                strcat(outBuf, outBufPID);
                lastOutP = outP;
                lastOutI = outI;
                lastOutD = outD;
                lastOutPID = outPID;
            }
                
            UART_dual_PutString(false);
        }
        
        
        // tipping detection
        if(compPitch > TIPPING_TRESHOLD || compPitch < -TIPPING_TRESHOLD) {
            sleep = true;
            motor_sate = MOTOR_SLEEP;            
            updateLED();
        }
        
        // motor states
        if(gyroStarted){
            switch(motor_sate){
                case MOTOR_BRAKE:
                    motor_curr_input = motor_input_sleep;
                    motor_resetPWM();
                    if(encoderL.currCount == 0 && encoderR.currCount == 0){
                        motor_sate = MOTOR_GO;       
                        encoder_clear(&encoderL);
                        encoder_clear(&encoderR);
                    }
                    break;
                case MOTOR_SLEEP:
                    motor_curr_input = motor_input_sleep;
                    motor_resetPWM();
                    break;
                default:
                    break;
            }
            
            motor_setDirection();
            motor_setPWM();
        }
    }
}

/* [] END OF FILE */
