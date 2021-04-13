//analogsensors.c - Touch Sensors functions

#include <hal_adc_async.h>
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

extern struct adc_async_descriptor         ADC_0;
extern struct adc_async_descriptor         ADC_1;



static void ADC_0_convert_cb(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
	uint8_t bytes_read;
	uint8_t buf_adc[4]; //a buffer to be able to read 2 channels with 12bit resolution
		
	//copy sample
	memset(buf_adc, 0x00, sizeof(buf_adc));
	bytes_read = adc_async_read_channel(&ADC_0, 0, (uint8_t *)&buf_adc, 4);

	AnalogSensor[4].Sample = (buf_adc[1] << 8) + buf_adc[0];
	AnalogSensor[5].Sample = (buf_adc[3] << 8) + buf_adc[2];

	printf("%x %x ",AnalogSensor[4].Sample,AnalogSensor[5].Sample);

//	battery_voltage_mv = adc_value_channel_0 * VREF_V_2V5 * BAT_DIV_V / RESOLUTION_12_BIT;
//	general_battery_voltage_mv = adc_value_channel_7 * VREF_V_2V5 * GENERAL_BAT_DIV_V / RESOLUTION_12_BIT;

}

static void ADC_1_convert_cb(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
	//copy sample
}



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
	adc_async_enable_channel(&ADC_0, 0); //AN4
	//adc_async_enable_channel(&ADC_0, 1);  //AN5
	adc_async_register_callback(&ADC_0, 0, ADC_ASYNC_CONVERT_CB, ADC_0_convert_cb);

	//enable ADC1 channels
	adc_async_enable_channel(&ADC_1, 0);
/*	adc_async_enable_channel(&ADC_1, 5);
	adc_async_enable_channel(&ADC_1, 6);
	adc_async_enable_channel(&ADC_1, 7);
	adc_async_enable_channel(&ADC_1, 10);
	adc_async_enable_channel(&ADC_1, 11);*/
	adc_async_register_callback(&ADC_1, 0, ADC_ASYNC_CONVERT_CB, ADC_1_convert_cb);



    memset(AnalogSensor,0,sizeof(AnalogSensorStatus)*NumAnalogSensors);
    
    for(i=0;i<NumAnalogSensors;i++) {
        AnalogSensor[i].Threshold=DEFAULT_ANALOG_THRESHOLD;
#if 0 
		switch(i) {
			case 4:
			case 5:
				AnalogSensor[i].ADC=&ADC_0;
				AnalogSensor[i].ADCNum=0;
				break;
			case 0:
			case 1:
			case 2:
			case 3:
			case 6:
			case 7:
				AnalogSensor[i].ADC=&ADC_1;
				AnalogSensor[i].ADCNum=1;
				break;
			default:
				printf("Unknown AnalogSensor %d\n",i);	
		} //switch(i)

#endif		
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
    uint8_t i,RegAddr;
    uint8_t ReturnValue,result;
	uint8_t bytes_read;
	uint8_t buf_adc[4]; //a buffer to be able to read 2 channels with 12bit resolution
 	  
	  
#if 0 	 
	//get sample
	memset(buf_adc, 0x00, sizeof(buf_adc));

	if ((AnalogSensor[4].flags&ANALOG_SENSOR_STATUS_ACTIVE) || (AnalogSensor[5].flags&ANALOG_SENSOR_STATUS_ACTIVE)) {						
		bytes_read = adc_async_read_channel(AnalogSensor[i].ADC, 0, (uint8_t *)&buf_adc, 4);

		AnalogSensor[4].Sample = (buf_adc[1] << 8) + buf_adc[0];
		AnalogSensor[5].Sample = (buf_adc[3] << 8) + buf_adc[2];

		printf("%x %x ",AnalogSensor[4].Sample,AnalogSensor[5].Sample);
	}
			
	SendAnalogSensorUDPPacket(); //send UDF packet with accel data
#endif
    return(1);
} //uint8_t Get_AnalogSensor_Samples(void) {


uint8_t SendAnalogSensorUDPPacket(void) {
	uint8_t *ReturnInst; //currently just 50 bytes but probably will change
	struct pbuf *retbuf;  //return buffer
	uint32_t ReturnInstLen, bufcount;
	uint8_t buffer[256]; //temporary buffer
	int i;

	//determine return buffer size
	bufcount=NumActiveAnalogSensors*2;  //16 bit samples
	
	retbuf = pbuf_alloc(PBUF_TRANSPORT, 5+bufcount, PBUF_RAM);
	ReturnInst=retbuf->payload;
	memcpy(ReturnInst,ASPStatus.ReturnIP,5); //copy IP + inst byte to return instruction
	

	bufcount=0;
    for(i=0;i<NumAnalogSensors;i++) {
	    if (AnalogSensor[i].flags&ANALOG_SENSOR_STATUS_ACTIVE) {
			ReturnInst[5+bufcount++]=AnalogSensor[i].Sample&0xff;
			ReturnInst[5+bufcount++]=AnalogSensor[i].Sample>>8;				
		} //AnalogSensor[i].flag&ANALOG_SENSOR_STATUS_ACTIVE
	} //for i
	

    if (bufcount>0) {
        //send the UDP packet
		udp_sendto(ASPStatus.pcb, retbuf, ASPStatus.addr, UDP_PORT); //dest port
		pbuf_free(retbuf);
      
    } //if (AccelTimerSendLen>5) {
    return(1);
}
