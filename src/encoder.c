#include "encoder.h"

#include <string.h>


Encoder encoderL;
Encoder encoderR;
unsigned int lastDecEncoderSpeedL = 0;
unsigned int lastDecEncoderSpeedR = 0;


void encoder_add(int a);


void encoder_init(Encoder *enc){
    memset(enc->buff, 0, ENCODER_BUFF_SIZE);
    enc->speed = 0;
    enc->currCount = 0;  
    enc->currIndex = 0;
}

void encoder_clear(Encoder *enc){   
    for (int i = 0; i < ENCODER_BUFF_SIZE; i++)
         enc->buff[i] = 0; 
        
    enc->currCount = 0;
    enc->speed = 0;
}

void encoder_calcSpeed(Encoder *enc){
    float sum = 0;    
    for (int i = 0; i < ENCODER_BUFF_SIZE; i++)
         sum += enc->buff[i]; 
        
    enc->speed = sum * ENCODER_INCREMENT_LENGTH_M * (10.0 / ENCODER_BUFF_SIZE);
}

void encoder_update(Encoder *enc){
    enc->buff[enc->currIndex] = enc->currCount;
    enc->currIndex++;
    if(enc->currIndex >= ENCODER_BUFF_SIZE)
        enc->currIndex = 0;
    enc->currCount = 0;
    
    encoder_calcSpeed(enc);
}