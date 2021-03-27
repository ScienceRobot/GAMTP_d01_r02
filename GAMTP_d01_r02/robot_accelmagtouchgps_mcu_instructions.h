//robot_accelmagtouchgps_mcu_instructions.h - instructions sent over UDP to MCU

//ACCEL+GYRO
#define DEFAULT_TIMER_INTERVAL 10;//50;//100 //ms was: 500 msec - timer 2+3 interval for accelerometer polling and all touch sensor sampling
#define DEFAULT_ACCEL_THRESHOLD 100//063 //126 //in mg

//USB_REQ 0-12 are already defined (see usb.h)
//perhaps there should be a "vendor request" - but perhaps using adress >=0xc0 is safe
//PCB 00-0f
#define ROBOT_ACCELMAGTOUCH_TEST 0x00 //send back 0x12345678
#define ROBOT_ACCELMAGTOUCH_PCB_NAME 0x01 //send back Name/ID of PCB ("Motor00", "Motor01", "Accel00", etc.)

//MCU 10-1f
#define ROBOT_ACCELMAGTOUCH_GET_MEM 0x11  //get PIC memory value
#define ROBOT_ACCELMAGTOUCH_SET_MEM 0x12  //set PIC memory value
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


