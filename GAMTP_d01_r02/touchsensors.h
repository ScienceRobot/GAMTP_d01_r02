//touchsensors.h
#ifndef TOUCHSENSORS_H
#define	TOUCHSENSORS_H

#include <stdint.h>  //for uint8_t

#define TOUCH_SENSOR_SEND_SIZE 50

#define MAX_NUM_TOUCH_SENSORS 8 //Maximum number of touch sensors this MCU can control
//pic32 10-bit samples #define DEFAULT_TOUCH_THRESHOLD 11//9
#define DEFAULT_TOUCH_THRESHOLD 44
//#define TOUCH_STATUS_MODE_POLL 0x0  //host will send a UDP packet to read this sensor- only send a UDP packet when requested
//#define TOUCH_STATUS_MODE_INTERRUPT 0x1 //send UDP packet whenever the threshold exceeded in this touch sensor
//TouchStatus defines the current touch sensor status for an array of touch sensors
#define TOUCH_SENSOR_STATUS_ACTIVE  0x1 //sensor is being read
#define TOUCH_SENSOR_STATUS_GOT_INITIAL_SAMPLE  0x2 //got at least one sample
//#define TOUCH_SENSOR_SINGLE_SAMPLE 0x4 //only get a single sample for this touch sensor
typedef struct {
    uint8_t    flags; //flags for accelerometer status
    uint16_t    Threshold; //threshold for this touch sensor (in interrupt mode, how much the value an change before sending a UDP packet, 255 allows a max change of .82v)
    uint16_t    SensorBit; // PORTB bit# to receive current sense on
    uint16_t    SensorBitMask; //mask to or/and portb pins with
    uint16_t    LastSample; //Last Sample- to compare the current sample to see if there was enough change to send a UDP packet (using touch sensor interrupt mode)
    uint16_t    Max; //Maximum Sample (voltage) recorded so far (used to calibrate touch sensor voltage range)
    uint16_t    Min; //Maximum Sample (voltage) recorded so far (used to calibrate touch sensor voltage range)
    volatile unsigned int *ADCBuf; //Touch sensor ADCBuffer address
} TouchSensorStatus;

uint8_t Initialize_TouchSensors(void);
uint8_t SetActiveTouchSensors(uint32_t mask,int Activate);

#endif //TOUCHSENSORS_H