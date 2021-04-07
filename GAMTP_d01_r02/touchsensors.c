//touchsensors.c - Touch Sensors functions

#include "touchsensors.h"
//#include "app.h"

extern uint8_t NumTouchSensors,NumActiveTouchSensors;
extern TouchSensorStatus TouchSensor[MAX_NUM_TOUCH_SENSORS]; //status of each touch sensor
extern uint8_t ActiveTouchSensor[MAX_NUM_TOUCH_SENSORS]; //list of all active touch sensors in order, for a quick reference
extern uint8_t TouchSensorSend[TOUCH_SENSOR_SEND_SIZE];  //touch sensor packet data to send back to requester

uint8_t Initialize_TouchSensors(void)
{
    uint8_t i;

    //clear the packet we send to whoever requests touch sensor data
    memset(TouchSensorSend,0,sizeof(TOUCH_SENSOR_SEND_SIZE));

    //Touch Sensors
    NumTouchSensors=8;
    NumActiveTouchSensors=0;
    //TRISB=0xffff;//portb is all inputs for touch sensor
    //ANSELB=0xffff; //set PORTB pins (also digital pins) to all analog, (all AN# pins are analog by default)

    //touch sensors:
    //0=b5,1=b4,2=b3,3=b2,4=b1,5=b0,6=b6...

    memset(TouchSensor,0,sizeof(TouchSensorStatus)*NumTouchSensors);
    for(i=0;i<6;i++) {
        TouchSensor[i].SensorBit=0x20>>i;
        //set buffer address where touch sensor data will be found
        //TouchSensor[i].ADCBuf=(unsigned int *)&ADCDATA0+(5-i);//*4; //add 4 bytes (+0x10) 32-bits
    }
    //TouchSensor[0].ADCBuf=&ADCDATA5;
    //TouchSensor[1].ADCBuf=&ADCDATA4;
    for(i=6;i<NumTouchSensors;i++) {
        TouchSensor[i].SensorBit=1<<i;
        //TouchSensor[i].ADCBuf=(unsigned int *)&ADCDATA0+i;//*4; //add 4 bytes (+0x10) 32-bits
    }
    //TouchSensor[7].ADCBuf=&ADCDATA7;
    
    for(i=0;i<NumTouchSensors;i++) {
        TouchSensor[i].Threshold=DEFAULT_TOUCH_THRESHOLD;
        //set min and max voltage for touch sensor to calibrate itself
        //note .Max is already set to 0
        //pic32mx is 10bit, pic32mz is 12bit = 0 to fff (/4096)
//        TouchSensor[i].Min=0x3ff;  //Set Min to highest value possible (10-bit ADC)
        //Currently I am going with 0.4 to 2.1v as min and max        
        //but these values can be set by the user
        //and are autotuned as the touch sensor is used (basically
        //if a lower or higher voltage is found, the min and max are changed,
        //which then changes the percent pressed, and percent change range)
        //conversion multiplier is: 
        //10-bit ADC sample, 0x3ff=1023.0  3.3/1023 = 0.003225806
        //12-bit ADC sample, 0xfff=4095.0  3.3/4096 = 0.000805
        TouchSensor[i].Min=0x1f0;//(496)  //0.4v pic23mz
        TouchSensor[i].Max=0xa2e;//(2606) //2.1v pic32mz
        

    } //for i
    //need SensorBitMask?


#if 0     
    //initialize Analog to Digital module
    ADCCON1=0; //in case adc is set on by harmony
    ADCCON2=0; //in case adc is set on by harmony
    ADCCON3=0; //in case adc is set on by harmony
    
    // Copy Calibration data
    ADC0CFG = DEVADC0;
    ADC1CFG = DEVADC1;
    ADC2CFG = DEVADC2;
    ADC3CFG = DEVADC3;
    ADC4CFG = DEVADC4;
    ADC7CFG = DEVADC7;    
    
     // ***see 16 motors for how to do this***
    //AD1PCFG&=0xbfff;    //make the RB14 pin analog
    //by setting its corresponding bit in the AD1PCFG register to 0
    //Set the analog pin as the Positive Input Select bits for Sample A Multiplexer Setting
    //AD1CHSbits.CH0SA=5;//mask which determines which ADC pin to read- not used for scanning
    //Set VREFL as the Negative Input Select bit for Sample A Multiplexer Setting
    //pic32mx AD1CHSbits.CH0NA=0;
    
    //capacitive voltage divider
    ADCCON2bits.CVDCPL = 3;//7; //17.5pF
    ADCCON1bits.CVDEN = 0;//1; //enable
    
    ADC0TIMEbits.ADCDIV = 1; // ADC0 clock frequency is half of control clock = TAD0
    ADC0TIMEbits.SAMC = 25;//3; // ADC0 sampling time = 5 * TAD0
    ADC0TIMEbits.SELRES = 3; // ADC0 resolution is 12 bits.    
    ADC1TIMEbits.ADCDIV = 1; // ADC1 clock frequency is half of control clock = TAD0
    ADC1TIMEbits.SAMC = 25;//3; // ADC1 sampling time = 5 * TAD1
    ADC1TIMEbits.SELRES = 3; // ADC1 resolution is 12 bits.    
    ADC2TIMEbits.ADCDIV = 1; // ADC2 clock frequency is half of control clock = TAD0
    ADC2TIMEbits.SAMC = 25;//3; // ADC2 sampling time = 5 * TAD2
    ADC2TIMEbits.SELRES = 3; // ADC2 resolution is 12 bits.    
    ADC3TIMEbits.ADCDIV = 1; // ADC3 clock frequency is half of control clock = TAD0
    ADC3TIMEbits.SAMC = 25;//3; // ADC3 sampling time = 5 * TAD3
    ADC3TIMEbits.SELRES = 3; // ADC3 resolution is 12 bits.    
    ADC4TIMEbits.ADCDIV = 1; // ADC4 clock frequency is half of control clock = TAD0
    ADC4TIMEbits.SAMC = 25;//3; // ADC4 sampling time = 5 * TAD4
    ADC4TIMEbits.SELRES = 3; // ADC4 resolution is 12 bits.    

    ADCCON1bits.FRACT=0; //integer (unsigned, single-ended are default) was: .FORM=0; //integer 16-bit format (10-bits)
    //pic32mx ADCCON1bits.SSRC=7; //(autoconvert) internal counter ends sampling and starts conversion
    //tph: Note that ASAM=1 is not mentioned in ADC Module Configuration
    //but is critical for getting a constant supply of samples
    //pic32mx ADCCON1bits.ASAM=1; //sampling begins immediately after last conversion
    ADCCON1bits.SELRES = 3; //pic32mz ADC7 resolution is 12 bits
    ADCCON1bits.STRGSRC = 1; //pic32mz Select scan trigger: Global software trigger (GSWTRG) is self-cleared on the next clock cycle
    ADCCON1bits.STRGLVL = 0;//1; // Scan trigger is high level sensitive. Once STRIG mode is selected (TRGSRCx<4:0> in the ADCTRGx
// register), the scan trigger will continue for all selected analog inputs, until the STRIG option is removed.
    ADCCON1bits.AICPMPEN = 0;  //disable charge pump

    //pic32mx ADCCON2bits.ALTS=0; //always use Sample A input multiplexer settings
    //a TAD is an ADC clock cycle- one for each bit plus 2 more
    ADCCON2bits.SAMC=25;//3;//25;//=27TAD sample time= 27TAD (12 TAD is typical for a conversion)
    //pic32mx ADCCON2bits.SAMC=25;//sample time= 25TAD (12 TAD is typical for a conversion)
    ADCCON2bits.ADCDIV=0x1;//0x7;//0x1;//0x7;//pic32mz 14*TQ=TAD7 14*640ns=8960ns TAD7=~9us
    //not clear why the interrupt needs to be enabled in ADCCON2
    //and how that works with the IEC6bits.ADCEOSIE=1 enable EOS intrpt bit
    ADCCON2bits.EOSIEN=1; //enable End of Scan interrupt 
    //pic32mx ADCCON3bits.ADRC=0; //clock derived from the Peripheral Bus Clock (PBCLK)
    //timing:
    ADCCON3bits.ADCSEL=0; //A2D clk source (TCLK) 0=use PBCLK 100MHz 3=use system clk 200MHz- use system clock for most accurate    
    ADCCON3bits.CONCLKDIV=1;//0x3f;//0;//0x3f; //64xTCLK=TQ 64*10ns=640ns    //A2D control clock (TQ) divider bits
    ADCCON3bits.VREFSEL=0; //VRef=AVdd and AVss (default)
    //PIC32MZ ADC timing:
    //max sample time=14TAD, max conversion time=13 TAD
    //we need a sample from 15 (~20) channels every 10ms.
    //10ms/20=500us
    //so if a single sample=14+13=27TAD, 500us/27=18us
    //TAD must be <=18us
    //PBCLK=100MHz (10ns)-> /CONCLKDIV -> /ADCDIV = TAD7 = 18us
    //
    //
    //PIC32MX ADC timing:
    //pic32mx ADCCON3bits.ADCS=0xff; //clock prescaler= 256 256TPB=TAD was 2 8TPB=TAD    
    //can be every 100ms (and sends 10 samples in a second)
    //so we want to mix sample time and TAD clock to finish each sample every 100ms
    //we need a TAD every 5ms (1s/10=0.1s /20TAD= .005s)
    //PB=80mhz 12.5ns * 20 TAD= 250ns 100,000,000/250=PB must be every 400,000 ns (400 us)
    //12.5ns * 256= 3.2us * 20 = 6.4us 15,625 interrupts in a second
    //slowest A2D using PB can be is 6.4us x 31TAD = 198.4us
    //currently 6.4us x 25 TAD = 625 interrupts every 100ms
    ADCCON3bits.RQCNVRT=0; //Individual ADC Input Conversion Request bit=do not trigger the conversion, pic32mz (default)

        
    ADCTRGMODEbits.SH0ALT=0;//an0 (default)
    ADCTRGMODEbits.SH1ALT=0;//an1 (default)
    ADCTRGMODEbits.SH2ALT=0;//an2 (default)
    ADCTRGMODEbits.SH3ALT=0;//an3 (default)
    ADCTRGMODEbits.SH4ALT=0;//an4 (default)
    
    
    /* Select ADC input mode */
    ADCIMCON1bits.SIGN0 = 0; //unsigned data format
    ADCIMCON1bits.DIFF0 = 0; //Single ended mode
    ADCIMCON1bits.SIGN1 = 0;
    ADCIMCON1bits.DIFF1 = 0;
    ADCIMCON1bits.SIGN2 = 0;
    ADCIMCON1bits.DIFF2 = 0;
    ADCIMCON1bits.SIGN3 = 0;
    ADCIMCON1bits.DIFF3 = 0;
    ADCIMCON1bits.SIGN4 = 0;
    ADCIMCON1bits.DIFF4 = 0;
    ADCIMCON1bits.SIGN5 = 0;
    ADCIMCON1bits.DIFF5 = 0;
    ADCIMCON1bits.SIGN6 = 0;
    ADCIMCON1bits.DIFF6 = 0;
    ADCIMCON1bits.SIGN7 = 0;
    ADCIMCON1bits.DIFF7 = 0;
    ADCIMCON1bits.SIGN8 = 0;
    ADCIMCON1bits.DIFF8 = 0;
    ADCIMCON1bits.SIGN9 = 0;
    ADCIMCON1bits.DIFF9 = 0;
    ADCIMCON1bits.SIGN10 = 0;
    ADCIMCON1bits.DIFF10 = 0;
    ADCIMCON1bits.SIGN11 = 0;
    ADCIMCON1bits.DIFF11 = 0;
    ADCIMCON1bits.SIGN12 = 0;
    ADCIMCON1bits.DIFF12 = 0;
    ADCIMCON1bits.SIGN13 = 0;
    ADCIMCON1bits.DIFF13 = 0;
    ADCIMCON1bits.SIGN14 = 0;
    ADCIMCON1bits.DIFF14 = 0;

    /* Configure ADCGIRQENx */
    //No individual ANx interrupts are used
    ADCGIRQEN1 = 0;
    ADCGIRQEN2 = 0;
    
    //pic23mx ADCCON2bits.VCFG=0; //VREFH=VDD VREFL=VSS
    //pic32mx ADCCON2bits.CSCNA=1; //1=scan inputs is enabled, 0=do not scan inputs
    //ADCCON1bits.STRGSRC; //scan trigger source select bits (pic32mz) 
    ADCTRG1bits.TRGSRC0=0x3; //STRIG, scan trigger, trigger source for AN0 select bits (pic32mz)
    ADCTRG1bits.TRGSRC1=0x3; //trigger source for AN1 select bits (pic32mz)
    ADCTRG1bits.TRGSRC2=0x3; //trigger source for AN2 select bits (pic32mz)
    ADCTRG1bits.TRGSRC3=0x3; //trigger source for AN3 select bits (pic32mz)
    ADCTRG2bits.TRGSRC4=0x3; //trigger source for AN4 select bits (pic32mz)
    ADCTRG2bits.TRGSRC5=0x3; //trigger source for AN5 select bits (pic32mz)
    ADCTRG2bits.TRGSRC6=0x3; //trigger source for AN6 select bits (pic32mz)
    ADCTRG2bits.TRGSRC7=0x3; //trigger source for AN7 select bits (pic32mz)
    ADCTRG3bits.TRGSRC8=0x3; //trigger source for AN8 select bits (pic32mz)    
    ADCTRG3bits.TRGSRC9=0x3; //trigger source for AN9 select bits (pic32mz)    
    ADCTRG3bits.TRGSRC10=0x3; //trigger source for AN10 select bits (pic32mz)    
    ADCTRG3bits.TRGSRC11=0x3; //trigger source for AN11 select bits (pic32mz)    
    //pic32mx ADCCON2bits.SMPI=NumTouchSensors; //interrupt after this many samples, 0=interrupt for each sample 
   


    /* Early interrupt */
    // No early interrupt  
    ADCEIEN1 = 0;
    ADCEIEN2 = 0;
    
    /* Configure ADCCMPCONx */
    //No digital comparators are used
    ADCCMPCON1 = 0;
    ADCCMPCON2 = 0;
    ADCCMPCON3 = 0;
    ADCCMPCON4 = 0;
    ADCCMPCON5 = 0;
    ADCCMPCON6 = 0;

    /* Configure ADCFLTRx */
    //No oversampling filters are used
    ADCFLTR1 = 0;
    ADCFLTR2 = 0;
    ADCFLTR3 = 0;
    ADCFLTR4 = 0;
    ADCFLTR5 = 0;
    ADCFLTR6 = 0;    

    ADCCSS2=0; //which higher analog inputs are scanned=none in the current EthAccel PCB
    
    /* Initialize warm up time register */
    ADCANCON = 0;
    ADCANCONbits.WKUPCLKCNT = 5; // Wakeup exponent = 32 * TADx



    ADCCON1bits.ON=1; //turn on ADC module - touch sensors are enabled by default
    
    
    /* Wait for voltage reference to be stable */
    while(!ADCCON2bits.BGVRRDY); // Wait until the reference voltage is ready
    while(ADCCON2bits.REFFLT);
       
    /* Enable clock to analog circuit */
    // Enable the clock to analog bias
    ADCANCONbits.ANEN7 = 1; // Enable clock, ADC7
    ADCANCONbits.ANEN0 = 1; // Enable clock, ADC0
    ADCANCONbits.ANEN1 = 1; // Enable clock, ADC1
    ADCANCONbits.ANEN2 = 1; // Enable clock, ADC2
    ADCANCONbits.ANEN3 = 1; // Enable clock, ADC3
    ADCANCONbits.ANEN4 = 1; // Enable clock, ADC4
    


    /* Wait for ADC to be ready */
    while(!ADCANCONbits.WKRDY7); // Wait until ADC7 is ready
    while(!ADCANCONbits.WKRDY0); // Wait until ADC0 is ready
    while(!ADCANCONbits.WKRDY1); // Wait until ADC1 is ready
    while(!ADCANCONbits.WKRDY2); // Wait until ADC2 is ready
    while(!ADCANCONbits.WKRDY3); // Wait until ADC3 is ready
    while(!ADCANCONbits.WKRDY4); // Wait until ADC4 is ready

    /* Enable the ADC modules */
    ADCCON3bits.DIGEN7 = 1; // Enable ADC7
    ADCCON3bits.DIGEN0 = 1; // Enable ADC0
    ADCCON3bits.DIGEN1 = 1; // Enable ADC1
    ADCCON3bits.DIGEN2 = 1; // Enable ADC2
    ADCCON3bits.DIGEN3 = 1; // Enable ADC3
    ADCCON3bits.DIGEN4 = 1; // Enable ADC4
    
    //pic32mx AD1 - ADC1 Convert Done interrupt

    //pic32mx IFS1bits..ADC1IF=0; //clear ADC1 interrupt flag
    //pic32mx IPC6bits.ADC1IP=4;  //set interrupt priority
    //pic32mx IPC6bits.ADC1IS=2;  //set interrupt subpriority
    //PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_END_OF_SCAN);
    //also can be: 
    IFS6bits.ADCEOSIF=0; //clear end of scan interrupt flag
    IPC48bits.ADCEOSIP=4;  //set interrupt priority
    IPC48bits.ADCEOSIS=2;  //set interrupt subpriority
//PLIB_INT_SourceFlagClear( INT_ID_0, INT_SOURCE_ADC_END_OF_SCAN );
//PLIB_INT_SourceEnable( INT_ID_0, INT_SOURCE_ADC_END_OF_SCAN );
//PLIB_INT_VectorPrioritySet( INT_ID_0, INT_VECTOR_ADC_END_OF_SCAN, INT_PRIORITY_LEVEL3 );
//PLIB_INT_VectorSubPrioritySet( INT_ID_0, INT_VECTOR_ADC_END_OF_SCAN, INT_SUBPRIORITY_LEVEL0 );
    //IEC1bits.AD1IE=1; //enable ADC1 interrupt
    //IEC6bits.ADCEOSIE=1; //enable end of scan interrupt

#endif

    return(1);
} //uint8_t Initialize_TouchSensors(void)



//activate or deactivate (Activate=0) individual touch sensors
//touch sensors may be activate or inactive by using a 32-bit mask
//if active=1 any 1s in the mask activate the sensor, 0s remain in the same state
//if active=0 any 1s in the mask inactivate the sensor, 0s remain in the same state
//flags are currently just to set the TouchSensor flag TOUCH_SENSOR_SINGLE_SAMPLE
//uint8_t SetActiveTouchSensors(uint32_t mask,int Activate,uint32_t flags)
uint8_t SetActiveTouchSensors(uint32_t mask,int Activate)
{
    uint8_t i,j,NumLower;

    //turn off ADC module because we change ADCCSS1
    //ADCCON1bits.ON=0; //turn off ADC module - touch sensors are disabled by default

    
    //reset number of touch sensors
    NumActiveTouchSensors=0;
    //ADCCSS1=0; //which analog inputs are scanned
    
    for(i=0;i<NumTouchSensors;i++) {
        
        if (mask&(1<<i)) {
            if (Activate) {
                TouchSensor[i].flags|=TOUCH_SENSOR_STATUS_ACTIVE;
                //because touch sensor # doesn't=PORTB pin#, they have to be remapped
                //ADCCSS1|=TouchSensor[i].SensorBit; //which AD channels to scan
                ActiveTouchSensor[NumActiveTouchSensors]=i;
                TouchSensor[i].flags&=~TOUCH_SENSOR_STATUS_GOT_INITIAL_SAMPLE;
                //TouchSensor[i].flags|=flagsl //currently only TOUCH_SENSOR_SINGLE_SAMPLE
                NumActiveTouchSensors++;
            } else {
                //deactivate this touch sensor and clear single sample flag              
                //TouchSensor[i].flags&=~(TOUCH_SENSOR_STATUS_ACTIVE|TOUCH_SENSOR_SINGLE_SAMPLE);                
                TouchSensor[i].flags&=~TOUCH_SENSOR_STATUS_ACTIVE;                
                //ADCCSS1&=~TouchSensor[i].SensorBit; //clear this AD channels from scanning
            }
        } else { //if (mask&(1<<i)) {
            //no mask, these sensors stay in the same state
            if (TouchSensor[i].flags&TOUCH_SENSOR_STATUS_ACTIVE) {
                //ADCCSS1|=TouchSensor[i].SensorBit; //which AD channels to scan
                ActiveTouchSensor[NumActiveTouchSensors]=i;
                NumActiveTouchSensors++;
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

} //uint8_t SetActiveTouchSensors(uint32_t mask,int Activate)


