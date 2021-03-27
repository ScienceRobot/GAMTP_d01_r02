//accel.h

#ifndef _ACCEL_H
#define _ACCEL_H

#include <stdint.h>  //for uint8_t
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

//#include "driver/driver_common.h" //for DRV_HANDLE
//#include "driver/i2c/drv_i2c.h" //for DRV_I2C_BUFFER_HANDLE and EVENT

//DOM-IGNORE-BEGIN
#ifdef	__cplusplus
extern "C" {
#endif
//DOM-IGNORE-END

#define TCA9548A_ADDRESS    0x70


    
#define MAX_NUM_ACCEL 8 //Maximum number of accelerometers this MCU can control
//#define ACCEL_POLL_INTERVAL 100 //check accelerometer every 100ms (10x a second)
//flags for SetActiveAccelerometers()
//#define ACCEL_FLAGS_DEACTIVATE 0x0 //for clarity - same as when ACCEL_FLAGS_ACTIVATE is not set
//#define ACCEL_FLAGS_ACTIVATE 0x1 //if 1, 1s in bit mask activate an individual accel, if 0 1s in bit mask deactivate an idividual accel, 0s in mask leave accel in the same state
//#define ACCEL_FLAGS_SINGLE_SAMPLE 0x2  //if 1, 1s in bit mask set the accel to get a single sample, if 0 1s in the bit mask clear the single sample flag, 0s in mask leave accel in same state
//#define ACCEL_FLAGS_POLLING 0x4 //if 1, 1s in bit mask activate polling for an individual accel, if 0 1s in bit mask deactivate polling for an individual accl, 0s in mask leave accel in the same state
//#define ACCEL_FLAGS_INTERRUPT 0x8 //if 1, 1s in bit mask activate interrupt for an individual accel, if 0 1s in bit mask deactivate interrupt for an individual accl, 0s in mask leave accel in the same state
//#define ACCEL_FLAGS_RELATIVE_ACCELERATION 0x10 //if 0 (absolute), an interrupt occurs when the acceleration if over (or under) a threshold value, if 1 (relative) an interrupt occurs when a change in acceleration goes over the threshold value

#define ACCEL_POLL_SEND_SIZE 80  //size of buffer that is sent to client with accel polling samples
#define ACCEL_INTR_SEND_SIZE 80  //size of buffer that is sent to client with accel interrupt samples
#define TOUCH_SEND_SIZE 80  //size of buffer that is sent to client with touch sensor samples

/* EthAccel Status*/
#define ACCEL_PCB_STATUS_TOUCH_SENSOR_POLLING 0x1 //polling one or more touch sensors
#define ACCEL_PCB_STATUS_TOUCH_SENSOR_INTERRUPT 0x2 //touch sensors interrupt
//todo: probably do away with the above flag, because touch sensor interrupts are polled with a timer enabling the adc interrupt
#define ACCEL_PCB_STATUS_ACCEL_POLLING 0x4 //polling one or more accelerometers
#define ACCEL_PCB_STATUS_ACCEL_INTERRUPT 0x8 //one or more accelerometers is set to send an interrupt
#define ACCEL_PCB_STATUS_GPS_ENABLED 0x10 //GPS is enabled
#define ACCEL_PCB_STATUS_SEND_ALL_GPS_DATA 0x20 //Send all GPS data is enabled
typedef struct {
	uint32_t flags;
}AccelPCBStatus;


//flags for individual accelerometers connected to this PCB
//AccelStatus defines the current accelerometer status for an array of accelerometers
#define ACCEL_STATUS_NOT_ENABLED 0x0 //for convenience to set all flags = 0
#define ACCEL_STATUS_ENABLED 0x1 //this accelerometer is enabled- only enabled accelerometers are sent I2C data (polled for samples)
#define ACCEL_STATUS_INITIALIZED 0x2 //this accelerometer has been initialized- sample rate, filters, etc. parameters set
#define ACCEL_STATUS_POLLING 0x4 //accelerometer is being polled (not sending interupts)
#define ACCEL_STATUS_SINGLE_SAMPLE 0x8 //getting only a single sample from accelerometer
#define ACCEL_STATUS_INTERRUPT 0x10 //accelerometer is set to send interrupts (not being polled, or set to get an individual sample)
#define ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION 0x20 //accel is set to send interrupt when relative acceleration exceeds the threshold (without this flag an interrupt is sent when the absolute acceleration exceeds the threshold)
#define ACCEL_STATUS_GOT_INTERRUPT 0x40 //this accelerometer recently sent an interrupt (in between timer 2+3 ticks)
#define ACCEL_STATUS_CLEAR_INTERRUPT 0x80 //this accelerometer recently sent an interrupt (in between timer 2+3 ticks)
#define ACCEL_STATUS_AUTOCALIBRATE 0x100 //accelerometer and magnetometer autocalibration enabled
typedef struct {
    uint16_t    flags; //flags for accelerometer status
    uint8_t     I2CAddress; //I2C Address- multiple accelerometers can be on 1 I2C bus
    uint32_t    I2CBus; //which I2CBus this accelerometer uses (=I2C1,I2C2, etc.)
    volatile unsigned int *PowerPort; //port with the pin that powers the accelerometer
    uint32_t PowerPinMask; //used to set/reset pin used to power accelerometers on the PowerPort 0x0080=portpin7
    uint8_t     IntNum; //which external interrupt this accel is connected to (0,1,2,3)
    uint16_t   Threshold; //(in mg) absolute or relative accel value that causes accel to interrupt
//    DRV_HANDLE handleI2C;  //device handle for this acclerometer
//    DRV_I2C_BUFFER_HANDLE  I2CBufferHandle;
//    DRV_I2C_BUFFER_EVENT   I2CBufferEvent;
    uint8_t Buffer[14]; //holds sample data
} AccelStatus;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

//prototypes
//uint8_t Power_On_Accelerometer(uint8_t AccelNum);
//uint8_t Power_Off_Accelerometer(uint8_t AccelNum);
uint8_t Reset_Accelerometer(uint8_t AccelNum);
uint8_t Initialize_Accelerometer(uint8_t AccelNum);
//uint8_t Accelerometer_StandByMode(uint8_t AccelNum);
//uint8_t Enable_Accelerometer_Interrupt(uint8_t num);
//uint8_t Disable_Accelerometer_Interrupt(uint8_t num);
uint8_t ConfigureAccelerometers(uint16_t mask,uint32_t flags,uint16_t Threshold);
//uint8_t Clear_Accelerometer_Interrupt(uint8_t num);
//uint8_t Process_Accelerometer_Interrupt(void);
uint8_t Get_Accelerometer_Samples(void);
uint8_t SendTimerUDPPacket(void);
uint8_t InitializeAccels(void);
#endif // _ACCEL_H