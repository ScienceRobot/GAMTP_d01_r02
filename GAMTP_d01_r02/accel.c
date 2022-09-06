//accelerometer.c
//Accelerometer (and gyroscope) functions

#include "accel.h"
//#include "app.h" //for delay_ms, ReadCoreTimer
#include "MPU6050.h"
#include <stdio.h>  //for printf
#include <string.h>
#include <hal_gpio.h>
#include <hal_delay.h>
#include <hal_timer.h>
#include <hal_i2c_m_sync.h>
#include <atmel_start_pins.h>
#include "robot_accelmagtouchgps_mcu_instructions.h"
#include <lwip/udp.h>
#include "main.h"
#include "analogsensors.h"

//Accel variables
uint8_t NumAccelerometers;//
AccelStatus Accel[MAX_NUM_ACCEL]; //status of each accelerometer
//uint8_t IntAccel[5];  //for ext int isr- which accel uses a given ext int
uint8_t AccelTimerSend[ACCEL_POLL_SEND_SIZE];  //accel polling and interrupt packet data to send back to requester
uint32_t AccelTimerSendLen; //length of accel polling and interrupt send data packet

//Touch Sensor variables:
//uint8_t NumTouchSensors,NumActiveTouchSensors;
//TouchSensorStatus TouchSensor[MAX_NUM_TOUCH_SENSORS]; //status of each touch sensor
//uint8_t ActiveTouchSensor[MAX_NUM_TOUCH_SENSORS]; //list of all active touch sensors in order, for a quick reference
//uint32_t A2DCount; //counts up ADC interrupts

//uint8_t TouchSensorSend[TOUCH_SENSOR_SEND_SIZE];  //touch sensor packet data to send back to requester
//uint32_t TouchSensorSendLen; //length of touch sensor send data packet
//#define GPS_SEND_SIZE 255
//uint8_t GPSSend[GPS_SEND_SIZE];  //touch sensor packet data to send back to requester
//uint32_t GPSSendLen; //length of touch sensor send data packet

AccelPCBStatus APStatus; //status of AccelTouch PCB

static struct timer_task AccelTimerTask;


extern AccelPCBStatus APStatus; 
extern uint8_t NumAccelerometers;
extern AccelStatus Accel[MAX_NUM_ACCEL];
extern struct timer_descriptor       TIMER_0;  
extern struct timer_descriptor       TIMER_1;
extern struct i2c_m_sync_desc        I2C_0;

extern uint8_t AccelTimerSend[ACCEL_POLL_SEND_SIZE];  //touch sensor packet data to send back to requester
extern uint32_t AccelTimerSendLen; //length of touch sensor send data packet

extern uint8_t TouchSend[TOUCH_SEND_SIZE];  //touch sensor packet data to send back to requester
extern uint32_t TouchSendLen; //length of touch sensor send data packet

extern AnalogSensorPCBStatus ASPStatus;

static void AccelTimerTask_cb(const struct timer_task *const timer_task)
{
//See if any accelerometers need samples, read samples, and send over UDP when done

    //Every TimerInterval, process:
    //Accel Polling- get (and send) samples from all enabled and polling accelerometers
    //Accel Interrupt- get (and send) samples from all enabled and interrupting accelerometers
    //Touch Sensor Polling and Interrupting
    
    if (APStatus.flags&ACCEL_PCB_STATUS_ACCEL_POLLING) {
		
	    Get_Accelerometer_Samples();
	} //if (APStatus.flags&ACCEL_PCB_STATUS_ACCEL_POLLING) {

	//I was processing interrupts in the ext int isrs,
	//but UDP packets are sent too quickly
	//Now interrupts are processed the same way polling samples are
	//because we are only checking for an interrupt every timer interrupt callback
	//interval (10ms))
	if (APStatus.flags&ACCEL_PCB_STATUS_ACCEL_INTERRUPT) {
		//Process_Accelerometer_Interrupt();
		Get_Accelerometer_Samples();
		    
	} //if (APStatus.flags&ACCEL_PCB_STATUS_ACCEL_INTERRUPT) {

	if (ASPStatus.flags&ACCEL_PCB_STATUS_ANALOG_SENSOR_POLLING)  {
		//Start ADC conversion (will call callback function once complete)
		//adc_async_start_conversion(&ADC_1);
		//adc_async_start_conversion(&ADC_0);
		Get_AnalogSensor_Samples();

		//adc_async_start_conversion(&ADC_1);
//		if (APStatus.flags&ACCEL_PCB_STATUS_ANALOG_SENSOR_SINGLE_SAMPLE) {
//			APStatus&=~ACCEL_PCB_STATUS_ANALOG_SENSOR_SINGLE_SAMPLE;
//		}
	} //
				


} //static void AccelTimerTask_cb(const struct timer_task *const timer_task)


//TIMER_1 uses the TC2 peripheral 
//which takes DPLL0 as input (120Mhz) divided down to 1Mhz, somehow the timer goes every 10ms (100Hz)
//was: which takes as input a 100kHz generic clock which creates an interrupt 
//every 10ms (100hz) to get readings and send over UDP
//NOTE: TIMER INTERRUPTS CAN DISRUPT DHCP
//NOTE: TC2 needs to be used for 32bit timers presumably use both TC0 and TC1
void AccelTimer_Initialize(void)
{
	AccelTimerTask.interval = 1; //clock ticks
	AccelTimerTask.cb       = AccelTimerTask_cb;
	AccelTimerTask.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_1, &AccelTimerTask);
	//timer_start(&TIMER_1);
}
		


/*
uint8_t Power_On_Accelerometer(uint8_t AccelNum)
{

    //power on the accelerometer
    *Accel[AccelNum].PowerPort|=Accel[AccelNum].PowerPinMask;
    delay_ms(1);

    return(1);
}

uint8_t Power_Off_Accelerometer(uint8_t AccelNum)
{
    //power off the accelerometer
    *Accel[AccelNum].PowerPort&=~Accel[AccelNum].PowerPinMask;
    delay_ms(1);

    return(1);
}
*/

uint8_t Reset_Accelerometer(uint8_t AccelNum)
{
    uint8_t I2CData[10],result,addr;    
	

    //this is a hardware reset- because that is the only way
    //to clear the accel I2C
//    Power_Off_Accelerometer(AccelNum);   
//    delay_ms(10);

    //power on the accelerometer
//    Power_On_Accelerometer(AccelNum);
//    delay_ms(10);


	//set TCA9548A mask to this accel
	//i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, TCA9548A_ADDRESS, I2C_M_SEVEN);
	addr=1<<AccelNum;  //select only the accel i2c channel
	io_write(&(I2C_0.io), &addr, 1);
	//io_read(&(I2C_0.io), mac, 6);
	delay_ms(50);  //was 5ms

    //followed by a software reset    
#if USE_MPU6050
    I2CData[0] = MPU6050_PWR_MGMT_1; //CNTL_REG2
    I2CData[1] = MPU6050_PWR_MGMT_1_DEVICE_RESET;  //Reset    
#endif //USE_MPU6050

	i2c_m_sync_set_slaveaddr(&I2C_0, MPU6050_ADDRESS, I2C_M_SEVEN);
	io_write(&(I2C_0.io), I2CData, 2);
    
//    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
//                                                        Accel[AccelNum].I2CAddress,
//                                                        I2CData,
//                                                        2, 
//                                                        NULL);


    //delay_ms(5); //definitely needed for I2C driver to start the 
    //transaction if no while after it because otherwise the i2c queue 
    //fills up. Need for waiting too- because complete may be returned before
    //inst is queued apparently.

    // Wait for the signal to complete
#if 0     
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);
    
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {
    switch(result) {
        case I2C_INST_TIMEOUT:
            printf("I2C write of Accel %d timed out. Disabling.\r\n",AccelNum);
            Power_Off_Accelerometer(AccelNum); 
            Accel[AccelNum].flags&=~ACCEL_STATUS_ENABLED;            
            return(0);
        break;
        case I2C_INST_ERROR: 
            printf("I2C write of Accel %d had error. Disabling.\r\n",AccelNum);
            Power_Off_Accelerometer(AccelNum); 
            Accel[AccelNum].flags&=~ACCEL_STATUS_ENABLED;
            return(0);
        break;
        default:
        break;
    } //switch
#endif

    delay_ms(100);  //wait for device to fully reset
    
    return(1);

} //uint8_t Reset_Accelerometer(uint8_t num)


uint8_t Initialize_Accelerometer(uint8_t AccelNum)
{
    uint8_t I2CData[10],addr;
    //uint8_t ReturnValue;
    //uint32_t Timeout;
	int32_t result;

    //start accelerometer

    //the firmware thinks there are 3 accels but some may be
    //disconnected so we want to make sure this failing doesn't
    //cause any problem

    
    if (!(Accel[AccelNum].flags&ACCEL_STATUS_ENABLED)) {
        //not enabled (but INITIALIZED flag can still be set)
        Accel[AccelNum].flags&=~ACCEL_STATUS_INITIALIZED;  //clear initialized flag
    }


	//set TCA9548A mask to this accel
	i2c_m_sync_set_slaveaddr(&I2C_0, TCA9548A_ADDRESS, I2C_M_SEVEN);
	addr=1<<AccelNum;  //select only the accel i2c channel
	printf("set I2C addr to %d\n",addr);
	result=io_write(&(I2C_0.io), &addr, 1);
	//printf("bytes written=%d\r\n",result);
    
#if USE_MPU6050

//#if 0 
    //enable digital low pass filter
    I2CData[0] = MPU6050_CONFIG;
    I2CData[1] = MPU6050_CONFIG_DLPF_CFG_1;

	i2c_m_sync_set_slaveaddr(&I2C_0, MPU6050_ADDRESS, I2C_M_SEVEN);
	result=io_write(&(I2C_0.io), I2CData, 2);
	//printf("bytes written=%d\r\n",result);

//    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
//                                                        Accel[AccelNum].I2CAddress,
//                                                        I2CData,
//                                                        1, 
//                                                        NULL);
/*
    delay_ms(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        printf("Write to MPU6050_CONFIG failed.\n\r");
        return(0);
    }
//#endif 
 */   

//#if 0 
//Sample rate divider    
    I2CData[0] = MPU6050_SMPRT_DIV;
    I2CData[1] = 4;//9;  //equals sample rate/1+9 = for accel 1khz/10 = 100hz

	i2c_m_sync_set_slaveaddr(&I2C_0, MPU6050_ADDRESS, I2C_M_SEVEN);
	io_write(&(I2C_0.io), I2CData, 2);

/*
    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        1, 
                                                        NULL);
    delay_ms(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        printf("Write to MPU6050_SMPRT_DIV failed.\n\r");
        return(0);
    }
//#endif 
  */  
    
    
    //take accel out of sleep/power save mode and bring into regular operation mode
    //and set clock to PLL with Z axis gyroscope reference  (0x3)
    I2CData[0] = MPU6050_PWR_MGMT_1;
    I2CData[1] = 3;//3;//0;


#endif //USE_MPU6050
    
	i2c_m_sync_set_slaveaddr(&I2C_0, MPU6050_ADDRESS, I2C_M_SEVEN);
	io_write(&(I2C_0.io), I2CData, 2);

	/*
    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    delay_ms(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result==I2C_INST_COMPLETE) {
//    if (WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
       // Accel[AccelNum].flags|=ACCEL_STATUS_ACTIVE; //set active flag
        Accel[AccelNum].flags|=ACCEL_STATUS_INITIALIZED; //set active flag        
        return(1);
    } else {
        return(0);
    }
*/

	//check WHO_AM_I
	I2CData[0] = MPU6050_WHO_AM_I;

	i2c_m_sync_set_slaveaddr(&I2C_0, MPU6050_ADDRESS, I2C_M_SEVEN);
	io_write(&(I2C_0.io), I2CData, 1);
	I2CData[0]=0;
	io_read(&(I2C_0.io), I2CData, 1);
	//i2c address: should be 0x68
	printf("Accel %d WHO_AM_I: 0x%x (0x68)\n",AccelNum, I2CData[0]);
	
	
	return(1);


}//uint8_t Initialize_Accelerometer(uint8_t AccelNum)


//Configure one or more accelerometers
//Note ENABLED flag needs to be included, or the accel selected will be disabled
//accelerometers may be enabled or disabled by using a 16-bit mask
//if enable=1 any 1s in the mask enable the sensor, 0s remain in the same state
//if enable=0 any 1s in the mask disable the sensor, 0s remain in the same state
//flags:
// - enable, polling, interrupt
//Threshold sets the interrupt threshold acceleration if >0
uint8_t ConfigureAccelerometers(uint16_t mask,uint32_t flags,uint16_t Threshold)
{
    uint8_t i,NumEnabledAccelerometers,NumPolling,NumInterrupting;

    //reset number of active accelerometer
    NumEnabledAccelerometers=0;    
    NumPolling=0;  
    NumInterrupting=0;
    

    for(i=0;i<NumAccelerometers;i++) {

        if (mask&(1<<i)) { //accel is selected
            //enable or disable the accel
            if (flags&ACCEL_STATUS_ENABLED) { //enable accel
                if (!(Accel[i].flags&ACCEL_STATUS_ENABLED)) { //Accel not already enabled
                    //power on and initialize accel
                    //Power_On_Accelerometer(i);
					Reset_Accelerometer(i);
                    if (!Initialize_Accelerometer(i)) {
                        //could not initialize accelerometer
                        //so disable it
                        printf("Accel %d cannot be initialized, disabling it\r\n",i);
                        Accel[i].flags&=~ACCEL_STATUS_ENABLED;                            
                    } else {
                        Accel[i].flags|=ACCEL_STATUS_ENABLED;
                        printf("Accel %d was enabled\r\n",i);
                    }                    
                } else { //if (!(Accel[i].flags&ACCEL_STATUS_ENABLED)) {
                    //Accel is already enabled
                    if ((Accel[i].flags&ACCEL_STATUS_INTERRUPT) && (flags&ACCEL_STATUS_POLLING)) {
                        //accel is changing from INTERRUPT to POLLING
                        //future: possibly disable the accel and PIC interrupts
                        //unset interrupt flag
                        Accel[i].flags&=~ACCEL_STATUS_INTERRUPT;
                        Accel[i].flags&=~ACCEL_STATUS_GOT_INTERRUPT;
                        //WaitForI2CBusIdle(Accel[i].I2CBus,TimerInterval*2);
                        //Disable_Accelerometer_Interrupt(i); //will deactivate
                        Reset_Accelerometer(i);
                        if (!Initialize_Accelerometer(i)) {
                            //return(0);
                        } //if (!Activate_Accelerometer(i)) {
                    } //if ((Accel[i].flags&ACCEL_STATUS_INTERRUPT) && (flags&ACCEL_STATUS_POLLING)) {
                } ////if (!(Accel[i].flags&ACCEL_STATUS_ENABLED)) {
                //ActiveAccel[NumEnabledAccelerometers]=i; //remember accel num
                if (flags&ACCEL_STATUS_INTERRUPT) { //set accel to interrupt
                    if (Threshold>0) { //user sent threshold
                        Accel[i].Threshold=Threshold; //in mg
                    } //if (Threshold>0) { //user sent threshold
                    if (Accel[i].flags&ACCEL_STATUS_POLLING) { //accel was polling
                        //disable the polling flag and wait 2*Timer for I2C bus to be idle
                        //in case a sample is in the process of being sent or received
                        Accel[i].flags&=~ACCEL_STATUS_POLLING;
                        delay_ms(1);
                        //WaitForI2CBusIdle(Accel[i].I2CBus,TimerInterval*2);                    
                    }//if (Accel[AccelNum].flags&ACCEL_STATUS_POLLING) {
                    
                    //enable the PIC external interrupt and accelerometer interrupt
                    Accel[i].flags|=ACCEL_STATUS_INTERRUPT;
                    if (flags&ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION) {  //have to set relative flag here for Enable_Accelerometer_Interrupt(i) function
                        Accel[i].flags|=ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION;
                    } else {
                        Accel[i].flags&=~ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION;
                    }  
                    //Enable_Accelerometer_Interrupt(i);
                    
                } //if (flags&ACCEL_STATUS_INTERRUPT) { //set accel to interrupt
                if (flags&ACCEL_STATUS_POLLING) { 
                    Accel[i].flags|=ACCEL_STATUS_POLLING;
                    if (flags&ACCEL_STATUS_SINGLE_SAMPLE) {
                        Accel[i].flags|=ACCEL_STATUS_SINGLE_SAMPLE;
                        //printf("Accel %d set to single sample\r\n",i);
                    }
                    
                }
            } else { //if (flags&ACCEL_STATUS_ENABLED) 
                //disable these accelerometers

                //To deactivate an accel, caution is needed
                //because the timer interrupt may be in the middle
                //of talking to the accel when the accel is disabled.              
                             
                if (Accel[i].flags&ACCEL_STATUS_INTERRUPT) {
                    //disable the PIC external interrupt and accelerometer interrupt
                    //Disable_Accelerometer_Interrupt(i);
                    Accel[i].flags&=~ACCEL_STATUS_INTERRUPT;
                    Accel[i].flags&=~ACCEL_STATUS_GOT_INTERRUPT;
                }
                //Accelerometer_StandByMode(i); //set accel to standby using i2c
                if (flags&ACCEL_STATUS_POLLING) { 
                    Accel[i].flags&=~ACCEL_STATUS_POLLING;
                    if (flags&ACCEL_STATUS_SINGLE_SAMPLE) {
                        Accel[i].flags&=~ACCEL_STATUS_SINGLE_SAMPLE;
                    }
                }
                delay_ms(1); //wait for all I2C communication to stop              
                Accel[i].flags&=~ACCEL_STATUS_ENABLED;
                //tph Power_Off_Accelerometer(i); 
            } //if (flags&ACCEL_STATUS_ENABLED) { //enable these accels
        } else { //if (mask&(1<<i)) {
            //sensor not in mask, disable these sensors
			Accel[i].flags&=~ACCEL_STATUS_ENABLED;
        } //if (mask&(1<<i)) {

        if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
            NumEnabledAccelerometers++;
        }
        if (Accel[i].flags&ACCEL_STATUS_POLLING) {
            NumPolling++;
        }
        if (Accel[i].flags&ACCEL_STATUS_INTERRUPT) {
            NumInterrupting++;
        }

    } //for i

    
    
    if (NumInterrupting>0) {
        APStatus.flags|=ACCEL_PCB_STATUS_ACCEL_INTERRUPT;
        //printf("APStatus interrupt\r\n");
    } else {
        APStatus.flags&=~ACCEL_PCB_STATUS_ACCEL_INTERRUPT;
    }
    if (NumPolling>0) {
        APStatus.flags|=ACCEL_PCB_STATUS_ACCEL_POLLING;
        //printf("APStatus polling\r\n");
    } else {
        APStatus.flags&=~ACCEL_PCB_STATUS_ACCEL_POLLING;
    }
    
    if (!NumEnabledAccelerometers) {
        //printf("no enabled accels- stopping APStatus interrupt and polling\r\n");
        //no active accelerometers clear poll and interrupt flags
        APStatus.flags&=~ACCEL_PCB_STATUS_ACCEL_INTERRUPT;
        APStatus.flags&=~ACCEL_PCB_STATUS_ACCEL_POLLING;        
/*        if (!(APStatus.flags&ACCEL_PCB_STATUS_TOUCH_SENSOR_POLLING) &&
            !(APStatus.flags&ACCEL_PCB_STATUS_TOUCH_SENSOR_INTERRUPT)) {
            //no interrupt or polling on touch sensors or accels, so stop timer
            T2CONbits.ON=0; //disable timer2+3 (for 32-bit)
        } else {
            T2CONbits.ON=1; //re-enable timer2+3 (for 32-bit)    
        }//
 */ 
//    } else {
        //enable the timer interrupt that checks for the accel interrupt flag
    //    T2CONbits.ON=1; //enable timer2+3 (for 32-bit)
        //the timer interrupt always stays enabled, but just in case enable again:
    //    IEC0bits.T3IE=1; //enable Timer 2+3 interrupt- doesn't enable the timer
    } //if (!NumEnabledAccelerometers) {
return(1);
} //uint8_t ConfigureAccelerometers(uint16_t mask,uint32_t flags,uint16_t Threshold)



//Get_Accelerometer_Samples
//This function is called by the (10ms) timer 
//to initiate: 1) building the return UDP packet)
//and 2) sending the first sample I2C transmit request
//was:
//builds and sends a return UDP packet with all the active
//accelerometer X,Y 12-bit samples in it
//flag can = ACCEL_STATUS_POLLING or ACCEL_STATUS_SINGLE_SAMPLE
//uint8_t Get_Accelerometer_Samples(uint32_t flag) {
uint8_t Get_Accelerometer_Samples(void) {
    uint8_t i,RegAddr,RegData;
    uint8_t ReturnValue,result;
    //uint32_t TempSendLen;
    uint8_t DonePolling;
    //printf("S\n\r");
  	uint8_t addr;  

#if 0 

//this causes sample read to fail
	//check WHO_AM_I
	addr=MPU6050_ADDRESS;
	RegAddr = MPU6050_WHO_AM_I;

	i2c_m_sync_set_slaveaddr(&I2C_0, MPU6050_ADDRESS, I2C_M_SEVEN);
	io_write(&(I2C_0.io), &RegAddr, 1);
	io_read(&(I2C_0.io), &RegData, 1);
	//byte returned: i2c address: should be 0x68
	printf("WHO_AM_I: 0x%x (0x68)\r\n",RegData);
#endif 	  
	  
	  
//#if 0 
    //TxSent=0;
    AccelTimerSendLen=5;
#if USE_MPU6050
    RegAddr= MPU6050_ACCEL_XOUT_H;  //start reg address for 14-byte bulk read
    //RegAddr= MPU6050_FIFO_R_W; //read from the FIFO
#endif     
    
    
    //determine which accel will be the last sample, in order for the 
    //I2C callback function to quickly and easily know to send the 
    //UDP packet (at each 10ms timer interrupt callback where there is
    //polling or an interrupt occurred)
 /*
    appData.LastAccelSample=2; //presume full 3 samples
    if (!(Accel[2].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_CLEAR_INTERRUPT))) {
        if (Accel[1].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_CLEAR_INTERRUPT)) {
            appData.LastAccelSample=1;
        } else {
            appData.LastAccelSample=0;
        } // if (Accel[1]
    }// if (!(Accel[2]
*/
    //freeze all interrupts here because an external interrupt
    //could occur while the for loop is running, and throw off the 
    //callback logic.
	/*
    for(i=0;i<NumAccelerometers;i++) {
        if (Accel[i].flags&ACCEL_STATUS_CLEAR_INTERRUPT) { 
            Accel[i].flags&=~ACCEL_STATUS_CLEAR_INTERRUPT;
            Accel[i].flags|=ACCEL_STATUS_GOT_INTERRUPT;
        } //if (Accel[i].flags
        
    } //for(i=0
*/
	
	DonePolling=1;
	
    for(i=0;i<NumAccelerometers;i++) {
        if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
            if (Accel[i].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_GOT_INTERRUPT)) {
				if (Accel[i].flags&ACCEL_STATUS_POLLING) {
					DonePolling=0; //still polling 
				}
                //set TCA9548A mask to this accel
				i2c_m_sync_set_slaveaddr(&I2C_0, TCA9548A_ADDRESS, I2C_M_SEVEN);
				addr=1<<i;
				io_write(&(I2C_0.io), &addr, 1);
				
				i2c_m_sync_set_slaveaddr(&I2C_0, MPU6050_ADDRESS, I2C_M_SEVEN);				
				io_write(&(I2C_0.io), &RegAddr, 1);
				io_read(&(I2C_0.io), Accel[i].Buffer, 14);
				//printf("%02x %02x %02x\n",Accel[i].Buffer[0],Accel[i].Buffer[1],Accel[i].Buffer[2]);
	
/*                Accel[i].I2CBufferHandle=DRV_I2C_TransmitThenReceive(Accel[i].handleI2C, 
                                                        Accel[i].I2CAddress,
                                                        &RegAddr, //byte to send: register addr
                                                        1,  
                                                        Accel[i].Buffer,//AccelTimerSend+TempSendLen,//AccelTimerSend+AccelTimerSendLen,//I2CData,
#if USE_MPU6050
                                                        14,
#endif                         
                                                        NULL); 
                DelayUs(500); //delay500uS
				*/
				if (Accel[i].flags&ACCEL_STATUS_SINGLE_SAMPLE) {
					//user only wants single sample, stop polling on this accelerometer
					Accel[i].flags&=~(ACCEL_STATUS_POLLING|ACCEL_STATUS_SINGLE_SAMPLE);	
				}
            } //if (Accel[i].flags&ACCEL_STATUS_POLLING) { //skip any set to interrupt
        } //if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
    } //for i

	if (DonePolling) {
		APStatus.flags&=~ACCEL_PCB_STATUS_ACCEL_POLLING;		
	}
	
	SendTimerUDPPacket(); //send UDF packet with accel data
    
    return(1);
} //uint8_t Get_Accelerometer_Samples(void) {


uint8_t SendTimerUDPPacket(void) {
	uint8_t *ReturnInst; //currently just 50 bytes but probably will change
	struct pbuf *retbuf;  //return buffer
	uint32_t ReturnInstLen, bufcount;
	uint8_t buffer[256]; //temporary buffer
	int i;

	//determine return buffer size
	bufcount=0;
	for(i=0;i<NumAccelerometers;i++) {
		if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
			bufcount+=15;  //1 byte AccelNum (1), 2 bytes*3=accel (6), 2 bytes temperature (2), 2*3=gyro (6)  
		} //if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
	} //for i	
	
	retbuf = pbuf_alloc(PBUF_TRANSPORT, 5+bufcount, PBUF_RAM);
	ReturnInst=retbuf->payload;
	memcpy(ReturnInst,APStatus.ReturnIP,5); //copy IP + inst byte to return instruction
	

	bufcount=0;
    for(i=0;i<NumAccelerometers;i++) {
	    if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
			ReturnInst[5+bufcount]=i; //accel num
			memcpy(&ReturnInst[6+bufcount],Accel[i].Buffer,14); //accel(16-bit*3)+temp(16-bit*1)+gyro(16-bit*3)
			bufcount+=15;
		} //if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
	} //for i
	

    if (bufcount>0) {
        //send the UDP packet
		udp_sendto(&APStatus.pcb, retbuf, &APStatus.addr, UDP_PORT); //dest port
		pbuf_free(retbuf);
      
    } //if (AccelTimerSendLen>5) {
    return(1);
}

uint8_t InitializeAccels(void) {

	int i;

	//set TC9548A ~reset~ pin = 1
	gpio_set_pin_direction(GPIO(GPIO_PORTA, 21),GPIO_DIRECTION_OUT);
	gpio_set_pin_level(GPIO(GPIO_PORTA, 21),true);
	delay_ms(1); //wait for pin level to reach 1

	NumAccelerometers=8; //there can be up to 8 accelerometers on a GAMTP
		
	for(i=0;i<NumAccelerometers;i++) {	 
		Accel[i].flags&=~ACCEL_STATUS_ENABLED;
	}
	//enable Accel[0]
	//Accel[0].flags|=ACCEL_STATUS_ENABLED;
		

	for(i=0;i<NumAccelerometers;i++) {
			
		//Initialize any enabled accelerometers
		if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
			//start each accelerometer
			if (Reset_Accelerometer(i)) {
				Accel[i].Threshold=DEFAULT_ACCEL_THRESHOLD; //set interrupt acceleration threshold (absolute or relative depending on flag)
				Accel[i].flags|=ACCEL_STATUS_AUTOCALIBRATE; //autocalibrate is on by default
				Initialize_Accelerometer(i);
				//Accel[i].flags|=ACCEL_STATUS_INITIALIZED; //testing

				// if accel was successfully initialized move to ready state
				if (Accel[i].flags&ACCEL_STATUS_INITIALIZED) {
					//  appData.i2cState[i] = APP_I2C_READY;
					printf("Initialized accelerometer %d.\r\n",i);
					} else {
					printf("Failed to initialize accelerometer %d.\r\n",i);
					//Accel[i].flags=ACCEL_STATUS_NOT_ENABLED;
				}
			} //if (Reset_Accelerometer(i)) {
		} //if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
	} //for i

	return(1);
}