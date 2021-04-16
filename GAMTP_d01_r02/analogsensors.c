//analogsensors.c - Touch Sensors functions

#include <stdio.h>
#include <string.h>
#include <hal_adc_sync.h>
#include <lwip/udp.h>
#include "main.h"
#include "analogsensors.h"
//#include "app.h"

//Analog Sensor variables:
uint8_t NumAnalogSensors,NumActiveAnalogSensors;
AnalogSensorStatus AnalogSensor[MAX_NUM_ANALOG_SENSORS]; //status of each analog sensor
uint8_t ActiveAnalogSensor[MAX_NUM_ANALOG_SENSORS]; //list of all active analog sensors in order, for a quick reference
AnalogSensorPCBStatus ASPStatus;

extern uint8_t NumAnalogSensors,NumActiveAnalogSensors;
extern AnalogSensorStatus AnalogSensor[MAX_NUM_ANALOG_SENSORS]; //status of each touch sensor
extern uint8_t ActiveAnalogSensor[MAX_NUM_ANALOG_SENSORS]; //list of all active touch sensors in order, for a quick reference
//extern uint8_t AnalogSensorSend[ANALOG_SENSOR_SEND_SIZE];  //touch sensor packet data to send back to requester

extern struct adc_sync_descriptor         ADC_0;
extern struct adc_sync_descriptor         ADC_1;


uint8_t Initialize_AnalogSensors(void)
{
    uint8_t i;

    //clear the packet we send to whoever requests touch sensor data
//    memset(AnalogSensorSend,0,sizeof(ANALOG_SENSOR_SEND_SIZE));

    //Touch Sensors
    NumAnalogSensors=8;
    NumActiveAnalogSensors=0;
    //touch sensors:

	//enable ADC0 channels
    adc_sync_enable_channel(&ADC_0, 0);
	//enable ADC1 channels
    adc_sync_enable_channel(&ADC_1, 0);
		
    memset(AnalogSensor,0,sizeof(AnalogSensorStatus)*NumAnalogSensors);
    
    for(i=0;i<NumAnalogSensors;i++) {
        AnalogSensor[i].Threshold=DEFAULT_ANALOG_THRESHOLD;
//#if 0 
		switch(i) {
			case 4:
			case 5:
				//AnalogSensor[i].ADC=&ADC_0;
				AnalogSensor[i].ADCNum=0;
				break;
			case 0:
			case 1:
			case 2:
			case 3:
			case 6:
			case 7:
				AnalogSensor[i].ADCNum=1;
				break;
			default:
				printf("Unknown AnalogSensor %d\n",i);	
		} //switch(i)

//#endif		

		AnalogSensor[0].ChannelNum=10;
		AnalogSensor[1].ChannelNum=11;
		AnalogSensor[2].ChannelNum=4;
		AnalogSensor[3].ChannelNum=5;
		AnalogSensor[4].ChannelNum=6;
		AnalogSensor[5].ChannelNum=1;
		AnalogSensor[6].ChannelNum=6;
		AnalogSensor[7].ChannelNum=7;

        //set min and max voltage for touch sensor to calibrate itself
        //note .Max is already set to 0
        //pic32mx is 10bit, pic32mz is 12bit = 0 to fff (/4096)
//        AnalogSensor[i].Min=0x3ff;  //Set Min to highest value possible (10-bit ADC)
        //Currently I am going with 0.4 to 2.1v as min and max        
        //but these values can be set by the user
        //and are autotuned as the touch sensor is used (basically
        //if a lower or higher voltage is found, the min and max are changed,
        //which then changes the percent pressed, and percent change range)
        //conversion multiplier is: 
        //10-bit ADC sample, 0x3ff=1023.0  3.3/1023 = 0.003225806
        //12-bit ADC sample, 0xfff=4095.0  3.3/4096 = 0.000805
        AnalogSensor[i].Min=0x1f0;//(496)  //0.4v pic32mz
        AnalogSensor[i].Max=0xa2e;//(2606) //2.1v pic32mz
        

    } //for i
    //need SensorBitMask?

    return(1);
} //uint8_t Initialize_AnalogSensors(void)



//activate or deactivate (Activate=0) individual touch sensors
//touch sensors may be activate or inactive by using a 32-bit mask
//if active=1 any 1s in the mask activate the sensor, 0s remain in the same state
//if active=0 any 1s in the mask inactivate the sensor, 0s remain in the same state
//flags are currently just to set the AnalogSensor flag TOUCH_SENSOR_SINGLE_SAMPLE
//uint8_t SetActiveAnalogSensors(uint32_t mask,int Activate,uint32_t flags)
uint8_t SetActiveAnalogSensors(uint32_t mask,int Activate)
{
    uint8_t i,j,NumLower;

    //turn off ADC module because we change ADCCSS1
    //ADCCON1bits.ON=0; //turn off ADC module - touch sensors are disabled by default

    
    //reset number of analog sensors
    NumActiveAnalogSensors=0;
    //ADCCSS1=0; //which analog inputs are scanned
    
    for(i=0;i<NumAnalogSensors;i++) {
        
        if (mask&(1<<i)) {
            if (Activate) {
                AnalogSensor[i].flags|=ANALOG_SENSOR_STATUS_ACTIVE;
                //because touch sensor # doesn't=PORTB pin#, they have to be remapped
                //ADCCSS1|=AnalogSensor[i].SensorBit; //which AD channels to scan
                ActiveAnalogSensor[NumActiveAnalogSensors]=i;
                AnalogSensor[i].flags&=~ANALOG_SENSOR_STATUS_GOT_INITIAL_SAMPLE;
                //AnalogSensor[i].flags|=flagsl //currently only TOUCH_SENSOR_SINGLE_SAMPLE
                NumActiveAnalogSensors++;
            } else {
                //deactivate this touch sensor and clear single sample flag              
                //AnalogSensor[i].flags&=~(TOUCH_SENSOR_STATUS_ACTIVE|TOUCH_SENSOR_SINGLE_SAMPLE);                
                AnalogSensor[i].flags&=~ANALOG_SENSOR_STATUS_ACTIVE;                
                //ADCCSS1&=~AnalogSensor[i].SensorBit; //clear this AD channels from scanning
            }
        } else { //if (mask&(1<<i)) {
            //no mask, these sensors stay in the same state
            if (AnalogSensor[i].flags&ANALOG_SENSOR_STATUS_ACTIVE) {
                //ADCCSS1|=AnalogSensor[i].SensorBit; //which AD channels to scan
                ActiveAnalogSensor[NumActiveAnalogSensors]=i;
                NumActiveAnalogSensors++;
            }
        } //if (mask&(1<<i)) {
        //note that the sample buffer number does not correspond to the
        //analog pin number- for example, if only AN6 is enabled
        //the first sample in the scan will be in ADC1BUF0 (not ADC1BUF6)
        //and so, if the next motor, motor[1] uses AN3, the ADC buffer for motor[0]
        //will change to ADC1BUF1- so for this reason all pins are sampled
        //and then the ADC1BUF corresponds to the pin number,
        //alternatively, earlier motors could be adjusted to the correct ADC1BUF address
    } //for i

} //uint8_t SetActiveAnalogSensors(uint32_t mask,int Activate)


//Get_AnalogSensor_Samples
//This function is called by the (10ms) timer 
//to 1) building the return UDP packet)
//and 2) reading the analog sensors
uint8_t Get_AnalogSensor_Samples(void) {
    uint8_t i;

	for (i=0;i<NumActiveAnalogSensors;i++) {
		switch(AnalogSensor[ActiveAnalogSensor[i]].ADCNum) {
			case 0: //ADC_0
			    adc_sync_set_inputs(&ADC_0,AnalogSensor[ActiveAnalogSensor[i]].ChannelNum, 0x18, 0);
			    adc_sync_read_channel(&ADC_0, 0,&AnalogSensor[ActiveAnalogSensor[i]].Sample, 2);
				//printf("0: %d: %x ",ActiveAnalogSensor[i],AnalogSensor[ActiveAnalogSensor[i]].Sample);
			break;
			case 1:  //ADC_1
			    adc_sync_set_inputs(&ADC_1,AnalogSensor[ActiveAnalogSensor[i]].ChannelNum, 0x18, 0);
			    adc_sync_read_channel(&ADC_1, 0,&AnalogSensor[ActiveAnalogSensor[i]].Sample, 2);
				//printf("1: %d: %x ",ActiveAnalogSensor[i],AnalogSensor[ActiveAnalogSensor[i]].Sample);
			break; 
		}
	} //for i
// 	printf("\n");  
	SendAnalogSensorUDPPacket(); //send UDF packet with analog sensor data
	  
    return(1);
} //uint8_t Get_AnalogSensor_Samples(void) {


uint8_t SendAnalogSensorUDPPacket(void) {
	uint8_t *ReturnInst; 
	struct pbuf *retbuf;  //return buffer
	uint32_t ReturnInstLen, bufcount;
	uint8_t buffer[256]; //temporary buffer
	int i;
	uint16_t Sample,LastSample,Threshold,AnalogSensorRange;
    float PercentPress,PercentChange;
	uint8_t ActiveSensor;

    if (NumActiveAnalogSensors>0) {

		//determine return buffer size
		bufcount=NumActiveAnalogSensors*4;  //8-bit %press, 8-bit % change, 16-bit samples
	
		retbuf = pbuf_alloc(PBUF_TRANSPORT, 5+bufcount, PBUF_RAM);
		ReturnInst=retbuf->payload;
		memcpy(ReturnInst,ASPStatus.ReturnIP,5); //copy IP + inst byte to return instruction
	
		//Analog Sensor data contains:
		//AnalogSensor# [0], %Press [1], %Change [2], Sample [3-4], etc. for as many sensors as are active
		//PercentPress is 0 to 255 which represents 0-100%
		//PercentChange is -128 to 127 which is 0-100%
		bufcount=5;
		for(i=0;i<NumActiveAnalogSensors;i++) {
			ActiveSensor=ActiveAnalogSensor[i];
			Sample=AnalogSensor[ActiveSensor].Sample;

			AnalogSensorRange=(AnalogSensor[ActiveSensor].Max-AnalogSensor[ActiveSensor].Min);
			PercentPress=(((float)Sample-(float)AnalogSensor[ActiveSensor].Min)/(float)AnalogSensorRange);
			//check for over limit
			if (PercentPress>1.0) {
				PercentPress=1.0;
			}

			//%Change ()
			PercentChange=(((float)Sample-(float)LastSample)/(float)AnalogSensorRange);
			//check for over and under limit            
			if (PercentChange>1.0) {
				PercentChange=1.0;
			}
			if (PercentChange<-1.0) {
				PercentChange=-1.0;
			}
		
			ReturnInst[bufcount++]=(uint8_t)(PercentPress*0xff);
			ReturnInst[bufcount++]=(uint8_t)(PercentChange*0xff);				
			memcpy(&ReturnInst[bufcount],(uint16_t *)&Sample,2);
			bufcount+=2;
		} //for i
	
		//send the UDP packet
		udp_sendto(&ASPStatus.pcb, retbuf, &ASPStatus.addr, UDP_PORT); //dest port
		pbuf_free(retbuf);
	} //if (NumActiveSensors)
    return(1);
}
