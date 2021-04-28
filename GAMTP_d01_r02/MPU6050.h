/* 
 * File:   MPU6050.h
 * Author: tedh
 *
 * Created on November 4, 2016, 4:00 AM
 */

#ifndef MPU6050_H
#define	MPU6050_H

#ifdef	__cplusplus
extern "C" {
#endif

#define MPU6050_ADDRESS    0x68

    //default full scale range for gyro is +-250 degrees/second, 
    //and for accel is +-2.0g
    
//14 bytes are read in a bulk read starting at MPU6050_ACCEL_XOUT_H
//ACCEL XH XL YH YL ZH ZL TEMPH TEMPL GYRO XH XL YH YL ZH ZL
    
#define MPU6050_SMPRT_DIV 0x19  //sample rate divider
#define MPU6050_CONFIG 0x1a      
#define MPU6050_CONFIG_DLPF_CFG_1 0x1 //Digital low-pass filter
//184hz accel (Fs=1khz), 188hz Gyro Fs=1khz  - at 10ms sampling=100hz, 
//(or =2  94hz accel, 98hz gyro) DLPF (3 LSBs)
#define MPU6050_FIFO_EN 0x23           
#define MPU6050_FIFO_EN_TEMP_FIFO_EN 0x80
#define MPU6050_FIFO_EN_XG_FIFO_EN 0x40
#define MPU6050_FIFO_EN_YG_FIFO_EN 0x20
#define MPU6050_FIFO_EN_ZG_FIFO_EN 0x10
#define MPU6050_FIFO_EN_ACCEL_FIFO_EN 0x08    
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_INT_PIN_CFG_INT_LEVEL 0x80  //int pin is active low if set
#define MPU6050_INT_PIN_CFG_LATCH_INT_EN 0x20  //0=50us pulse, 1=set until cleared
#define MPU6050_INT_PIN_CFG_INT_RD_CLEAR 0x10  //0=must read INT_STATUS reg, 1=int is cleared on any read
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_ENABLE_DATA_RDY_EN 0x01 //1=int when all sensor regs have been written to
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_PWR_MGMT_1 0x6b  //controls DEVICE_RESET[7], SLEEP[6], CYCLE[5], TEMP_DIS[3], and CLKSEL[2:0]
    //clearing the SLEEP bit in MPU6050_PWR_MGMT_1     
     //takes the sensor out of sleep mode and in to regular operation mode.
    //CLKSEL 0=8MHz, 3=PLL w Z axis gyroscope reference
#define MPU6050_FIFO_R_W 0x74  //FIFO Read/Write
#define MPU6050_PWR_MGMT_1_DEVICE_RESET 0x80  //sets DEVICE_RESET bit

    
    
    
#define MPU6050_WHOAMI    0x75

#ifdef	__cplusplus
}
#endif

#endif	/* MPU6050_H */

