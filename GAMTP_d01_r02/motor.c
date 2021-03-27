//motor.c


#include "motor.h"
#include <hal_gpio.h>
#include <hal_delay.h>
#include <hal_timer.h>
#include "robot_motor_mcu_instructions.h"

static struct timer_task MotorTimerTask;

//Motor variables
uint8_t NumMotors;
MotorStatus Motor[MAX_NUM_MOTORS];  //status of each motor
int NumClocksInMotorDutyCycle; //the number of timer interrupt clocks in the motor duty cycle
int MotorDutyCycleClock; //the time (in us) of each timer2 interrupt when the motor pins are updated


extern struct timer_descriptor       TIMER_0;



static void MotorTimerTask_cb(const struct timer_task *const timer_task)
{
//Process any motor pwm
	uint8_t temp;
	uint8_t i;
	int MotorActive;



	MotorActive=0;
	//go through each motor status and set/reset the correct PORTB and PORTC pins
	for(i=0;i<NumMotors;i++) {//NUM_MOTORS;i++) {
		if (Motor[i].Duration>0) { //this motor is moving, or will start moving
			MotorActive=1;
			//set strength duty cycle
			if (Motor[i].StrengthCount<Motor[i].Strength && !(Motor[i].flags&MOTOR_INST_FIRST_RUN)) { //set motor pins
				//in "on" portion of duty cycle
				//clear and set the direction pins
				//(is the same for driver with or without a pulse pin)
				if (Motor[i].Direction) {
					//todo: write directly to correct OUTSET/OUTCLR register
					gpio_set_pin_level(Motor[i].DirPin,true);
					//CCW
										
					//clear clockwise pin
					//LATB&=Motor[i].DirectionCWBitMask;
					//*Motor[i].DirCWPort&=Motor[i].DirectionCWBitMask;
				} else {
					gpio_set_pin_level(Motor[i].DirPin,false);
					//CW
					//clear counter-clockwise pin
					//set clockwise pin
					//*Motor[i].DirCWPort|=Motor[i].DirectionCWBit;
				}//if (Motor[i].Direction) {

				//pulse the pulse pin if necessary
				gpio_set_pin_level(Motor[i].PulsePin,true);
			} else {  //if (Motor[i].StrengthCount<Motor[i].Strength)
			
				//in "off" part of duty cycle (or first run of a motor instruction)

				if (Motor[i].flags&MOTOR_INST_FIRST_RUN) {
					Motor[i].flags&=~MOTOR_INST_FIRST_RUN; //clear first run bit
					if (Motor[i].StrengthCount>0) {
						Motor[i].StrengthCount--; //set back 1 so direction can get set above
					}
				} //if (Motor[i].flags&MOTOR_INST_FIRST_RUN) {
									    
				//Note that there is no need to set direction pins
				//because the "off" duty cycle is never called before the "on"
				//duty cycle, and the "on" duty cycle sets the direction
				//which doesn't need to change for the "off" cycle
					
				//*Motor[i].PulsePort&=Motor[i].PulseBitMask;
				gpio_set_pin_level(Motor[i].PulsePin,false);
			} //if (Motor[i].StrengthCount<motor[i].Strength)

			Motor[i].StrengthCount++;  //increase duty cycle count
			if (Motor[i].StrengthCount>=NumClocksInMotorDutyCycle) {  //reached end of duty cycle, reset count
				Motor[i].StrengthCount=0;
			}

			//decrement Duration
			if (Motor[i].Duration!=0) { //new inst might set Duration=0
				Motor[i].Duration--;  //currently units are timer interrupts
			}
			//which depend on MotorDutyCycleClock
		} else {  //no motor Duration - set motor pins to 0

			Motor[i].StrengthCount=0;
									    
			gpio_set_pin_level(Motor[i].PulsePin,false);
			} //if (Motor[i].Duration>0)
		} //for(i=0;i<NUM_MOTORS;i++)

	if (MotorActive==0) {
		timer_stop(&TIMER_0);  //done with timer_0
	}

} //static void MotorTimerTask_cb(const struct timer_task *const timer_task)


//TIMER_0 uses the TC0 peripheral which takes as input a 2MHz generic clock which creates an interrupt every 25us- a 40khz (25us) signal for Motor pwm
void MotorTimer_Initialize(void)
{
	MotorTimerTask.interval = 1; //clock ticks
	MotorTimerTask.cb       = MotorTimerTask_cb;
	MotorTimerTask.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &MotorTimerTask);
	//timer_start(&TIMER_0);
}
		
		
//this instruction is a motor instruction to execute now
//and contains (in 4 bytes):
//Motor# (0:3),unused(4:7),direction(8),thrust(9:15),duration(16:31)
//was 3 bytes:
//motor# (address):4, direction:1, strength:3,duration:16
void SendMotorInst(uint8_t *MInst)
{
    uint8_t MotorNum,Strength;
    uint32_t Duration;


    //set the motor status so the timer interrupt
    //will find that a motor has a duration and needs to be moved
    //determine which motor
    MotorNum=(MInst[0]&0xf0)>>4;

    Motor[MotorNum].Duration=0; //just in case motor is currently running (pulsing)
    //SYS_TMR_DelayMS(1);  //wait to make sure motor is stopped if running (pulsing)
    Motor[MotorNum].flags|=MOTOR_INST_FIRST_RUN; //first set motor pins to 00 to avoid short circuit
    //for example, motor is turning with 10 then a 01 instruction is sent, and for a nanosecond when LATB is being set, possibly 2 pins might be 1 and cause a short at the h-bridge.

    //if ((MInst[0]&0x08)!=0) {
    if ((MInst[1]&0x80)!=0) {
        //counter-clockwise
       // Motor[MotorNum].DirectionMask=0x40>>(MotorNum*2);  //counter-clockwise
         //Motor[MotorNum].DirectionMask=0x1<<(MotorNum*2);  //counter-clockwise
         Motor[MotorNum].Direction=MOTOR_COUNTERCLOCKWISE;
    } else {
        //clock-wise
        //Motor[MotorNum].DirectionMask=0x80>>(MotorNum*2);  //clockwise
        //Motor[MotorNum].DirectionMask=0x2<<(MotorNum*2);  //clockwise
        Motor[MotorNum].Direction=MOTOR_CLOCKWISE;
    }

    //set motor turn strength- number of timer clocks in "on" part of duty cycle
    //0=stop 20=full speed (NumClkInDutyCycle)
    //this number is multiplied by the NumClocksInMotorDutyCycle/20
    //so for strength=1, Strength*NumClocksInMotorDutyCycle/20 (1 clks of 20 are on)
    //for strength=20 the motor is on for every clock (20 of 20)
    //Motor[MotorNum].Strength=((MInst[0]&0x07)*NumClocksInMotorDutyCycle)/ROBOT_MOTORS_DEFAULT_NUM_CLKS_IN_MOTOR_DUTY_CYCLE;
    Strength=(MInst[1]&0x7f);//max is currently 0 to 127
    //if strength is higher than the number of possible speeds, set at maximum strength
    if (Strength> ROBOT_MOTORS_DEFAULT_NUM_CLKS_IN_MOTOR_DUTY_CYCLE) {
        Strength=ROBOT_MOTORS_DEFAULT_NUM_CLKS_IN_MOTOR_DUTY_CYCLE;
    }

    //set motor turn duration (is 16-bit little endian int)
    //Motor[MotorNum].Duration=(uint16_t)((MInst[2]<<8)+MInst[1]);
    Duration=(uint32_t)((MInst[3]<<8)+MInst[2]);
    //duration from user is in ms, so convert to timer interrupts:
    //Motor[MotorNum].Duration is in timer interrupts 
    //MotorDutyCycleClock is 25us by default (40khz)  = every 25us
    //so convert Duration into time interrupt units (how many 25us units)

    //if Duration*1000 < 127*MotorDutyCycleClock then Strength needs to be scaled down
    //because the entire duty cycle of the motor will be < 127.
    //ex: DurTime=1ms, MDCC=25us DurInts=1000/25=only 40 so if strength=127/2=63, that needs to be scaled down to 63/127=x/40
    //x=63*40/127=19 so 19 timer interrupts will be on, and 21 off for a total of 40x25us=1ms

    //convert ms to 25us units
    //so Duration of 100ms in 25uS units=0.1/0.000025=4000 clocks * 2= 8000
    //in us: (100)*1000/25=4000, so generalizing in us:
    //Duration*1000/MotorDutyCycle = number of TimerInterrupts for 
    //user sent Duration in ms.
    Duration*=1000; //Duration is divided by MotorDutyCycleClock below
    //MotorDutyCycleClock is in ms so multiply Duration x 1000 
	
    //at 40khz (25us) a pulse:
	//duty cycle is 25us*20=500us
	//so 1/20 speed is 25us on 475us off (shortest pulse possible for drv8800- 40khz)
	//   19/20 speed is 475us on 25us off

    Motor[MotorNum].Strength=Strength;
    Motor[MotorNum].StrengthCount=0;

    //Convert duration in us to duration in number of timer interrupts
    //since multiplying by 1000 above, I doubt Duration would ever be < MotorDutyCycleClock
    //but just as a failsafe in case Motor[].Duration somehow will get set to 0]
    //and somebody is trying to set strength=0 - probably not needed
    if ((Duration/MotorDutyCycleClock)==0 && Duration>0) {
      Motor[MotorNum].Duration=1;
    } else {
      Motor[MotorNum].Duration=Duration/MotorDutyCycleClock;  //note: this is the equivalent of enabling the motor
    } 

    Motor[MotorNum].DurationCount=0;

	//start Motor PWM timer if not started already
	if (!_timer_is_started(&TIMER_0)) {
		timer_start(&TIMER_0);
	}
	
} //SendMotorInst

int InitializeMotors(void)
{
	
	int i;
	
	//set number of clocks in motor duty cycle
	//is 7 (but was 14), 7 timer2 interrupts make 1 full motor duty cycle
	NumClocksInMotorDutyCycle=ROBOT_MOTORS_DEFAULT_NUM_CLKS_IN_MOTOR_DUTY_CYCLE;
	MotorDutyCycleClock=ROBOT_MOTORS_DEFAULT_MOTOR_DUTY_CYCLE_CLK;
	
	
	NumMotors=16;
	//Clear the robot status array
	memset(Motor,sizeof(MotorStatus)*NumMotors,0);

	//Motor[0].flags|=MOTOR_DRIVER_USES_PULSE_PIN;
	Motor[0].DirPin=GPIO(GPIO_PORTB, 13);
	Motor[0].PulsePin=GPIO(GPIO_PORTB, 12);
	Motor[1].DirPin=GPIO(GPIO_PORTB, 11);
	Motor[1].PulsePin=GPIO(GPIO_PORTB, 10);
	Motor[2].DirPin=GPIO(GPIO_PORTA, 11);
	Motor[2].PulsePin=GPIO(GPIO_PORTA, 10);
	Motor[3].DirPin=GPIO(GPIO_PORTA, 9);
	Motor[3].PulsePin=GPIO(GPIO_PORTA, 8);

	Motor[4].DirPin=GPIO(GPIO_PORTA, 7);
	Motor[4].PulsePin=GPIO(GPIO_PORTA, 6);
	Motor[5].DirPin=GPIO(GPIO_PORTA, 5);
	Motor[5].PulsePin=GPIO(GPIO_PORTA, 4);
	Motor[6].DirPin=GPIO(GPIO_PORTB, 9);
	Motor[6].PulsePin=GPIO(GPIO_PORTB, 8);
	Motor[7].DirPin=GPIO(GPIO_PORTB, 7);
	Motor[7].PulsePin=GPIO(GPIO_PORTB, 6);
	
	Motor[8].DirPin=GPIO(GPIO_PORTB, 3);
	Motor[8].PulsePin=GPIO(GPIO_PORTB, 2);
	Motor[9].DirPin=GPIO(GPIO_PORTB, 1);
	Motor[9].PulsePin=GPIO(GPIO_PORTB, 0);
	Motor[10].DirPin=GPIO(GPIO_PORTC, 25);
	Motor[10].PulsePin=GPIO(GPIO_PORTC, 24);
	Motor[11].DirPin=GPIO(GPIO_PORTB, 25);
	Motor[11].PulsePin=GPIO(GPIO_PORTB, 24);

	Motor[12].DirPin=GPIO(GPIO_PORTB, 21);
	Motor[12].PulsePin=GPIO(GPIO_PORTB, 20);
	Motor[13].DirPin=GPIO(GPIO_PORTB, 19);
	Motor[13].PulsePin=GPIO(GPIO_PORTB, 18);
	Motor[14].DirPin=GPIO(GPIO_PORTB, 17);
	Motor[14].PulsePin=GPIO(GPIO_PORTB, 16);
	Motor[15].DirPin=GPIO(GPIO_PORTC, 19);
	Motor[15].PulsePin=GPIO(GPIO_PORTC, 18);

	
	
	for(i=0;i<NumMotors;i++) {
		gpio_set_pin_direction(Motor[i].DirPin,GPIO_DIRECTION_OUT);
		gpio_set_pin_level(Motor[i].DirPin,false);
		gpio_set_pin_direction(Motor[i].PulsePin,GPIO_DIRECTION_OUT);
		gpio_set_pin_level(Motor[i].PulsePin,false);
	}

	return(1);
} //int InitializeMotors(void)

