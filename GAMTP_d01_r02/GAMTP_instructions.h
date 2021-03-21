//GAMTP_instructions.h - instructions sent over UDP to MCU

//here because are used in Robot program running on main CPU
#define ROBOT_MOTORS_DEFAULT_MOTOR_DUTY_CYCLE_CLK 25 //25us 40khz above human hearing range, was 1000us 1ms is the timer2 interrupt that sets or clears the motor pulse pins
#define ROBOT_MOTORS_DEFAULT_NUM_CLKS_IN_MOTOR_DUTY_CYCLE	20//7 //a single duty cycle = 7 clocks (at 1/7th speed, the motor is on for only 1 of the 7 clocks, for full speed the motor pins are set on all 7 of 7 clocks)
//num clocks in motor duty cycle is equal to the number of speeds a motor can have. But too many speeds make the duty cycle (the time
//where the motor is turned on and then off for one cycle) too long.
//examples: (duty cycle clock=25us)
//1) num clks in duty cycle=127, 1/10 speed = 12on 115off = 300us on 2875us off (~3ms off would not make for smooth motion)
//2) num clks in duty cycle= 40, 1/10 speed =  4on  36off = 100us on  900us off (~1ms off is still pretty long)
//3) num clks in duty cycle= 20, 1/10 speed =  2on  18off =  50us on  450us off 
//4) num clks in duty cycle= 14, 1/10 speed =  1on  13off =  25us on  325us off
//5) num clks in duty cycle=  7, 1/10 speed =  1on   6off =  25us on  150us off  (the original number of duty cycles {speeds})


//USB_REQ 0-12 are already defined (see usb.h)
//perhaps there should be a "vendor request" - but perhaps using address >=0xc0 is safe
//PCB instructions 0x00-0x0f
#define ROBOT_MOTORS_TEST 0x00 //send back 0x12345678
#define ROBOT_MOTORS_PCB_NAME 0x01 //send back Name/ID of PCB ("Motor00", "Motor01", "Accel00", etc.)
//PIC instructions 0x10-0x1f
#define ROBOT_MOTORS_GET_MEM 0x10  //get PIC memory value
#define ROBOT_MOTORS_SET_MEM 0x11  //set PIC memory value
//Motor instructions 0x20-0xff
#define ROBOT_MOTORS_SEND_4BYTE_INST 0x20  //send 4 Byte Instruction to execute now
#define ROBOT_MOTORS_GET_4BYTE_DATA 0x21  //get 4 Byte Returned Data
#define ROBOT_MOTORS_NEW_PROGRAM 0x22  //send stored program instruction start time
#define ROBOT_MOTORS_SEND_PROGRAM_INST_STARTTIME 0x23  //send stored program instruction start time
#define ROBOT_MOTORS_SEND_PROGRAM_INST 0x24  //send stored program instruction
#define ROBOT_MOTORS_RUN_PROGRAM 0x25  //run the stored program
#define ROBOT_MOTORS_STOP_PROGRAM 0x26  //stop the stored program running and start at beginning again
#define ROBOT_MOTORS_STEP_PROGRAM 0x27  //Execute the next instruction in the stored program
#define ROBOT_MOTORS_PAUSE_PROGRAM 0x28  //pause the running stored program and resume
#define ROBOT_MOTORS_CONTINUE_PROGRAM 0x29  //continue paused stored program
#define ROBOT_MOTORS_GET_MOTOR_DUTY_CYCLE_CLK_IN_USEC 0x2a //get the timer2 interrupt interval- default=1ms- alternatively PR2 can be directly set with ROBOT_SET_REG
#define ROBOT_MOTORS_SET_MOTOR_DUTY_CYCLE_CLK_IN_USEC 0x2b //set the timer2 interrupt interval- default=1ms- alternatively PR2 can be directly set with ROBOT_SET_REG
#define ROBOT_MOTORS_GET_NUM_CLK_IN_MOTOR_DUTY_CYCLE 0x2c  //get the num of timer2 interrupts in a motor duty cycle default=14
#define ROBOT_MOTORS_SET_NUM_CLK_IN_MOTOR_DUTY_CYCLE 0x2d  //set the num of timer2 interrupts in a motor duty cycle default=14
#define ROBOT_MOTORS_SEND_CURRENT_SENSE 0x2e  //send motor current sense A2D reading(s)
#define ROBOT_MOTORS_STOP_SENDING_CURRENT_SENSE 0x2f  //stop sending motor current sense A2D readings
#define ROBOT_MOTORS_STOP_ALL_MOTORS 0x30 //stop all motors
//WIFI (ESP8266/ESP-01)
#define ROBOT_MOTORS_TEST_WIFI 0x40 //test to see if wifi esp-01 module is connected and responding
//
//ACCEL+GYRO
#define DEFAULT_TIMER_INTERVAL 10;//50;//100 //ms was: 500 msec - timer 2+3 interval for accelerometer polling and all touch sensor sampling
#define DEFAULT_ACCEL_THRESHOLD 100//063 //126 //in mg

//USB_REQ 0-12 are already defined (see usb.h)
//perhaps there should be a "vendor request" - but perhaps using adress >=0xc0 is safe
//PCB 00-0f
//#define ROBOT_ACCELMAGTOUCH_TEST 0x00 //send back 0x12345678
//#define ROBOT_ACCELMAGTOUCH_PCB_NAME 0x01 //send back Name/ID of PCB ("Motor00", "Motor01", "Accel00", etc.)
//MCU 10-1f
//#define ROBOT_ACCELMAGTOUCH_GET_MEM 0x11  //get PIC memory value
//#define ROBOT_ACCELMAGTOUCH_SET_MEM 0x12  //set PIC memory value
#define ROBOT_ACCELMAGTOUCH_GET_TIMER_INTERVAL_IN_MSEC 0x13 //get the current timer interrupt interval (in ms)
#define ROBOT_ACCELMAGTOUCH_SET_TIMER_INTERVAL_IN_MSEC 0x14  //set the timer interrupt interval (in ms)

//ACCEL 50-6f
#define ROBOT_ACCELMAGTOUCH_GET_ACCEL_REG 0x50
#define ROBOT_ACCELMAGTOUCH_SET_ACCEL_REG 0x51
#define ROBOT_ACCELMAGTOUCH_RESET_ACCELEROMETER 0x52
#define ROBOT_ACCELMAGTOUCH_GET_ACCELEROMETER_VALUES 0x53 //sends a UDP packet with a sample for each active accelerometer (note: accels need to be set active before this call)
#define ROBOT_ACCELMAGTOUCH_START_POLLING_ACCELEROMETER 0x54 //sends a UDP packet when there is a large enough change in acceleration
#define ROBOT_ACCELMAGTOUCH_STOP_POLLING_ACCELEROMETER 0x55
#define ROBOT_ACCELMAGTOUCH_START_ACCELEROMETER_INTERRUPT 0x56
#define ROBOT_ACCELMAGTOUCH_STOP_ACCELEROMETER_INTERRUPT 0x57
#define ROBOT_ACCELMAGTOUCH_GET_ACCELEROMETER_INTERRUPT_THRESHOLD 0x58 //get accelerometer threshold- how much change in acceleration (in mg) until an interrupt (and UDP packet)is sent
#define ROBOT_ACCELMAGTOUCH_SET_ACCELEROMETER_INTERRUPT_THRESHOLD 0x59 //set accelerometer threshold
#define ROBOT_ACCELMAGTOUCH_ENABLE_ACCELMAG_AUTOCALIBRATION 0x5a  //enable autocalibration on the accelerometer+magnetometer
#define ROBOT_ACCELMAGTOUCH_DISABLE_ACCELMAG_AUTOCALIBRATION 0x5b  //enable autocalibration on the accelerometer+magnetometer
#define ROBOT_ACCELMAGTOUCH_GET_HARD_IRON_OFFSET 0x5c  //get the hard iron offset the magnetometer is using
#define ROBOT_ACCELMAGTOUCH_SET_HARD_IRON_OFFSET 0x5d  //set the hard iron offset the magnetometer will use

//TOUCH 70-8f
#define ROBOT_ACCELMAGTOUCH_ENABLE_TOUCH_SENSORS 0x70  //enable the ADC interrupt
#define ROBOT_ACCELMAGTOUCH_DISABLE_TOUCH_SENSORS 0x71  //disable the ADC interrupt
#define ROBOT_ACCELMAGTOUCH_GET_TOUCH_SENSOR_VALUES 0x72  //read the voltage on touch sensors (0-3ff=0-3.3v)
#define ROBOT_ACCELMAGTOUCH_START_POLLING_TOUCH_SENSORS 0x73  //enable the timer interrupt, send sample every 100ms
#define ROBOT_ACCELMAGTOUCH_STOP_POLLING_TOUCH_SENSORS 0x74  //disable the timer interrupt
#define ROBOT_ACCELMAGTOUCH_START_TOUCH_SENSORS_INTERRUPT 0x75  //enable the ADC interrupt, send sample whenever large change (depending on threshold) occurs
#define ROBOT_ACCELMAGTOUCH_STOP_TOUCH_SENSORS_INTERRUPT 0x76  //disable the ADC interrupt
#define ROBOT_ACCELMAGTOUCH_GET_TOUCH_SENSOR_THRESHOLD 0x77 //get the threshold for 1 or more touch sensor
#define ROBOT_ACCELMAGTOUCH_SET_TOUCH_SENSOR_THRESHOLD 0x78 //set the threshold for 1 or more touch sensor (.1v=
//(1 bit=0.003225806v, .1v=31 0x1f)
#define ROBOT_ACCELMAGTOUCH_GET_TOUCH_MINMAX 0x7b //get the min and max Voltage for 1 or more touch sensors
#define ROBOT_ACCELMAGTOUCH_SET_TOUCH_MINMAX 0x7c  //set the min and max Voltage for 1 or more touch sensors

//GPS 90-9f
#define ROBOT_ACCELMAGTOUCH_GET_GPS_DATA 0x90  //start sending GPS data
#define ROBOT_ACCELMAGTOUCH_STOP_GPS_DATA 0x91  //stop sending GPS data
#define ROBOT_ACCELMAGTOUCH_SET_SEND_ALL_GPS_DATA 0x92  //set 'send all GPS data' flag
#define ROBOT_ACCELMAGTOUCH_UNSET_SEND_ALL_GPS_DATA 0x93  //unset 'send all GPS data' flag
#define ROBOT_ACCELMAGTOUCH_ENABLE_STATIC_NAVIGATION 0x94  //
#define ROBOT_ACCELMAGTOUCH_DISABLE_STATIC_NAVIGATION 0x95  //


