#include <stdbool.h>


#define ENCODER_BUFF_SIZE 5
#define ENCODER_INCREMENT_LENGTH_M 0.01021


typedef struct Encoder{
    float speed;
    int currCount; // current interrupt count    
    int currIndex; 
    int buff[ENCODER_BUFF_SIZE];
} Encoder;

extern Encoder encoderL;
extern Encoder encoderR;
extern unsigned int lastDecEncoderSpeedL;
extern unsigned int lastDecEncoderSpeedR;

extern volatile bool encoderEvalReady;


void encoder_init(Encoder *enc);

void encoder_update(Encoder *enc);

void encoder_clear(Encoder *enc);