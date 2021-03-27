#include <atmel_start.h>
#include <hal_gpio.h>
#include <hal_delay.h>
#include <driver_examples.h>
#include <peripheral_clk_config.h>
#include <ethernet_phy_main.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>
#include <lwip/udp.h>
#include <lwip/timers.h>
#include <lwip_demo_config.h>
#include <stdio.h>
#include <string.h>
#include <udpserver.h>
#include <robot_motor_mcu_instructions.h>
#include <robot_accelmagtouchgps_mcu_instructions.h>
#include <main.h>
#include <motor.h>  //motor data structures (motor port values and masks, duration, and program data structure)
#include <accel.h>  //accelerometer data structures ()
#include <utils_ringbuffer.h> //needed for esp-01 usart

#define PCB_NAME_LENGTH 5
static const char *PCB_Name = "GAMTP";  //Motor
#define UDP_PORT 53510  //port used for UDP communication


/* Saved total time in mS since timer was enabled */
volatile static u32_t systick_timems;
volatile static bool  gmac_recv_flag = false;
static bool           link_up   = false;
u8_t    mac[6];
static bool got_ip = false;

struct udp_pcb *udpserver_pcb; //udp server
//extern struct mac_async_descriptor MACIF; //is declared as ETHERNET_MAC_0 in driver_init.c
extern struct mac_async_descriptor ETHERNET_MAC_0;
extern struct netif LWIP_MACIF_desc;
extern u8_t LWIP_MACIF_hwaddr[6];
//extern struct io_descriptor *stdio_io; 
extern uint8_t AccelTimerSend[ACCEL_POLL_SEND_SIZE];  //accel polling and interrupt packet data to send back to requester
extern uint8_t NumAccelerometers;//
extern AccelStatus Accel[MAX_NUM_ACCEL]; //status of each accelerometer
extern AccelPCBStatus APStatus; //status of AccelTouch PCB



u32_t sys_now(void)
{
	return systick_timems;
}

void SysTick_Handler(void)
{
	systick_timems++;
}

void systick_enable(void)
{
	systick_timems = 0;
	SysTick_Config((CONF_CPU_FREQUENCY) / 1000);
}


// Last entry in ring buffer (ring buffer found at io_descr->rx in callback).
// Can be used for parsing in the call back.
uint8_t ringbuffer_last(struct ringbuffer const *const rb)
{
    ASSERT(rb);

    uint8_t data = rb->buf[(rb->write_index-1) & rb->size];

    return data;
}

volatile bool line_available = false;
//int NumCharRx=0;
void usart1_receive_cb(const struct usart_async_descriptor *const io_descr)
{
//	struct io_descriptor *io_in,*io_out; 
//	u8_t buffer[256];
//	int NumChars;
	
	/* Read transfer completed */
	//if (io_descr->rx.size>14) {
	//	line_available=true;
	//}
	//Note that the ring buffer can add characters after a new line before being read from
    if( ringbuffer_last(&io_descr->rx) == 0x0a ) {
        line_available = true;
		//NumCharRx=io_descr->rx.size;
	}
	
	//printf("u1 ");
//	usart_async_get_io_descriptor(&USART_1, &io_in);
//	usart_sync_get_io_descriptor(&USART_1, &io_out);
	//CRITICAL_SECTION_ENTER()
//	if (usart_async_is_rx_not_empty(&USART_1)) {
//	printf("%s",&io_descr->rx.buf[io_descr->rx.read_index]);
		//NumChars=ringbuffer_num(&io_descr->rx);
//		NumChars=io_read(io_in, (uint8_t *)&buffer,255);
//		buffer[NumChars]=0; //terminal string
		//io_write(io_out, (uint8_t *)&buffer,1);
//		printf("%s",buffer);
//	} //if (usart_async_is_rx_not_empty(USART_1)) {
	//CRITICAL_SECTION_LEAVE()

}

int32_t nread = 0;
static uint8_t buffer[256];
void USART_1_input(void)
{
    struct io_descriptor *io,*io_out;
    bool willRead = false;
    usart_async_get_io_descriptor(&USART_1, &io);	// Get the pointer to statically allocated io_descriptor structure.
	uint8_t MotorInst[4];

    if( nread <= 0 ) {
        // Here need a semaphore (counts only to 1 in interrupt):
        CRITICAL_SECTION_ENTER()
        if(line_available) {
            willRead = true;			// Accepting the semaphore count.
            line_available = false;		// Decrementing the semaphore count from 1 to 0.
        }								// Fast in and out with interrupts disabled.
        CRITICAL_SECTION_LEAVE()
        if (willRead) {
            //nread = io_read(io, buffer, sizeof(buffer));	// Returns count of characters read (or negative error flag).
			nread = io_read(io, buffer, sizeof(buffer));	// Returns count of characters read (or negative error flag).
			//nread=io_read(io,buffer,NumCharRx); //only read to 0x0a- leave anything else in ring buffer
			if (nread>0) {
				//usart_sync_get_io_descriptor(&USART_0, &io_out);	// Get the pointer to statically allocated io_descriptor structure.
				//io_write(io_out,buffer,1);
				//printf("W: %s",buffer); //each USART packet ends with 0x0a
				buffer[nread]=0; //terminate string
				printf("W: %s:W\n",buffer); //each USART packet ends with 0x0a
				//note that there is no transmit buffer (even for the synchronous driver) printf calls can overwrite the buffer currently

				//tread+=nread; //incremement total read count
				nread=0;
				//if (tread>=9) { //end of motor instruction length
				//Process any motor instructions
				if (buffer[4]==ROBOT_MOTORS_SEND_4BYTE_INST) {
					//got send a 4 byte Instruction over USART from ESP-01
					MotorInst[0]=buffer[5];  //Motor Num<<4
					MotorInst[1]=buffer[6];  //Dir+Strength
					MotorInst[2]=buffer[7];  //duration low byte
					MotorInst[3]=buffer[8];  //duration high byte
					SendMotorInst(MotorInst);
					//tread-=9;
				} //if (buffer[4]==ROBOT_MOTORS_SEND_4BYTE_INST) {
				//} //if (tread==9) {
			} //if (nread>0) {
		} //if(willRead) {
    } //if( nread <= 0 ) {
}

void usart0_receive_cb(const struct usart_async_descriptor *const io_descr)
{
}


void mac_receive_cb(struct mac_async_descriptor *desc)
{
	gmac_recv_flag = true;
	//printf("rx ");
	gpio_set_pin_level(PHY_YELLOW_LED_PIN,false);
	delay_ms(1);
	//gpio_set_pin_level(PHY_YELLOW_LED_PIN,false);
	gpio_set_pin_level(PHY_YELLOW_LED_PIN,true);
}

void mac_transmit_cb(struct mac_async_descriptor *desc)
{
	//gmac_tx_flag = true;
	//printf("tx ");
	gpio_set_pin_level(PHY_YELLOW_LED_PIN,false);
	delay_ms(1);
	//gpio_set_pin_level(PHY_YELLOW_LED_PIN,false);
	gpio_set_pin_level(PHY_YELLOW_LED_PIN,true);
}

static void status_callback(struct netif *n)
{
	if (n->flags & NETIF_FLAG_UP) {
		printf("Interface Up %s:\n",
		n->flags & NETIF_FLAG_DHCP ? "(DHCP)" : "(STATIC)");

		#if 0
		printf("  IP Address: " IP_F "\n", IP_ARGS(&n->ip_addr));
		printf("  Net Mask:   " IP_F "\n", IP_ARGS(&n->netmask));
		printf("  Gateway:    " IP_F "\n", IP_ARGS(&n->gw));

		const char *speed = "10Mb/s";
		if (ETH->MACCR & ETH_MACCR_FES)
		speed = "100Mb/s";

		const char *duplex = "Half";
		if (ETH->MACCR & ETH_MACCR_DM)
		duplex = "Full";

		printf("  Mode:       %s  %s Duplex\n", speed, duplex);
		#endif
		} else {
		printf("Interface Down.\n");
	}
}

static void link_callback(struct netif *n)
{

	if (n->flags & NETIF_FLAG_LINK_UP) {
		printf("Link Up.\n");

		if (n->flags & NETIF_FLAG_DHCP) {
			printf("Restarting DHCP\n");
			dhcp_start(n);
		}

		} else {
		printf("Link Down.\n");
	}
}


static void print_ipaddress(void)
{
	static char tmp_buff[16];
	printf("IP_ADDR    : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *)&(LWIP_MACIF_desc.ip_addr), tmp_buff, 16));
	printf("NET_MASK   : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *)&(LWIP_MACIF_desc.netmask), tmp_buff, 16));
	printf("GATEWAY_IP : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *)&(LWIP_MACIF_desc.gw), tmp_buff, 16));
}

static void read_macaddress(u8_t *mac)
{
	
	//struct i2c_m_sync_desc I2C_0;
#if CONF_AT24MAC_ADDRESS != 0
	uint8_t addr = 0x9A;
/*	i2c_m_sync_enable(&I2C_AT24MAC);
	i2c_m_sync_set_slaveaddr(&I2C_AT24MAC, CONF_AT24MAC_ADDRESS, I2C_M_SEVEN);c	
	io_write(&(I2C_AT24MAC.io), &addr, 1);
	io_read(&(I2C_AT24MAC.io), mac, 6);*/
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, CONF_AT24MAC_ADDRESS, I2C_M_SEVEN);
	io_write(&(I2C_0.io), &addr, 1);
	io_read(&(I2C_0.io), mac, 6);

#else
	/* set mac to 0x11 if no EEPROM mounted */
	//memset(mac, 0x11, 6);
	mac[0]=0x74;
	mac[1]=0x27;
	mac[2]=0xea;
	mac[3]=0xda;
	mac[4]=0x89;
	mac[5]=0x85;
#endif
}

#if 0 
//Process any kind of instruction - instructions are generalized so that this code is the same whether the instruciton came from ethernet, usart, or usart-esp
void Process_Instruction(uint8_t Inst,uint32_t len,uint8_t type)
{

} //void Process_Instruction(struct pbuf *p)
#endif		
		
		
		
void udpserver_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
	int i;
	uint8_t *InstData; //pointer to udp data (instruction)
	uint8_t *ReturnInst; //currently just 50 bytes but probably will change
	struct pbuf *retbuf;  //return buffer
	int InstLen,ReturnInstLen;
	uint8_t MotorInst[4];  //motor instruction
	struct io_descriptor *io; //for ESP-01 UART1
	uint8_t buffer[256]; //temporary buffer
	uint16_t Sample,Threshold,AccelMask,AccelThreshold;
		
	//printf("received at %d, echoing to the same port\n",pcb->local_port);
	//dst_ip = &(pcb->remote_ip); // this is zero always
	if (p != NULL) {
		//printf("UDP rcv %d bytes: ", (*p).len);
		printf("%d ", (*p).len);
		//    	  for (i = 0; i < (*p).len; ++i)
		//			printf("%c",((char*)(*p).payload)[i]);
		//    	printf("\n");
		//udp_sendto(pcb, p, IP_ADDR_BROADCAST, 1234); //dest port
				//		udp_sendto(pcb, p, &forward_ip, fwd_port); //dest port
				
		//Process any UDP instructions recognized
		if (pcb->local_port==UDP_PORT) {  //note that currently there could never be a different port because UDP server only listens to this port
			//printf("port: %d\n", pcb->local_port);
			
			InstData=(uint8_t *)(*p).payload;  //shorthand to data
			InstLen=(*p).len;
			switch(InstData[4]) //Robot Instruction
			{
			case ROBOT_MOTORS_TEST: //send back 0x12345678
				retbuf = pbuf_alloc(PBUF_TRANSPORT, 10, PBUF_RAM);
				ReturnInst=retbuf->payload;
				memcpy(ReturnInst,p->payload,5); //copy IP + inst byte to return instruction
				ReturnInst[6]=0x12;
				ReturnInst[7]=0x34;
				ReturnInst[8]=0x56;
				ReturnInst[9]=0x78;
				udp_sendto(pcb, retbuf, addr, UDP_PORT); //dest port
				pbuf_free(retbuf);
				break;
			case ROBOT_MOTORS_PCB_NAME: //01 send back mac+name/id
				retbuf = pbuf_alloc(PBUF_TRANSPORT, 5+sizeof(LWIP_MACIF_hwaddr)+PCB_NAME_LENGTH, PBUF_RAM);
				ReturnInst=retbuf->payload;
				ReturnInstLen=5;				   
				memcpy(ReturnInst,p->payload,5); //copy IP + inst byte to return instruction
				//get the MAC address from the default network interface
				//this presumes that there is only 1 net
				//in the future there could be more than 1 net,
				//for example a wireless net too
				memcpy(ReturnInst+ReturnInstLen,LWIP_MACIF_hwaddr,sizeof(LWIP_MACIF_hwaddr));//copy mac
				ReturnInstLen+=sizeof(LWIP_MACIF_hwaddr);
				memcpy(ReturnInst+ReturnInstLen,PCB_Name,PCB_NAME_LENGTH);//copy name
				ReturnInstLen+=PCB_NAME_LENGTH; //MOTOR
				udp_sendto(pcb, retbuf, addr, UDP_PORT); //dest port
				pbuf_free(retbuf);
			break;
	        case ROBOT_MOTORS_SEND_4BYTE_INST:  //got send a 4 byte Instruction over Net (was USB)
				MotorInst[0]=InstData[5];  //Motor Num<<4
				MotorInst[1]=InstData[6];  //Dir+Strength
				MotorInst[2]=InstData[7];  //duration low byte
				MotorInst[3]=InstData[8];  //duration high byte
				SendMotorInst(MotorInst);
			break; 
			case ROBOT_MOTORS_TEST_WIFI:				
				usart_async_get_io_descriptor(&USART_1, &io);
				memcpy(buffer,&InstData[5],InstLen-5);
				buffer[InstLen-5]=0; //terminate string
				printf("%s",buffer);
				io_write(io, (uint8_t *)&InstData[5], InstLen-5);
			break;
			//**************
			//Accelerometers
			//**************
			case ROBOT_ACCELMAGTOUCH_GET_ACCELEROMETER_VALUES:
				printf("Get accel values");
				memcpy(AccelTimerSend,InstData,5); //copy IP + inst byte to return instruction
				memcpy(&AccelMask,(uint16_t *)&InstData[5],2);  //copy 16-bit mask
				//printf("AccelMask=%d\r\n",AccelMask);
				//ACCEL_STATUS_POLLING needs to be set currently because the sample is retrieved by the timer ISR
				if (APStatus.flags&ACCEL_PCB_STATUS_ACCEL_INTERRUPT) {
					for(i=0;i<NumAccelerometers;i++) {
						if (Accel[i].flags&ACCEL_STATUS_INTERRUPT) {
							//trigger a sample send in the next timer interrupt callback
							Accel[i].flags|=ACCEL_STATUS_CLEAR_INTERRUPT;
						}
					} //for i
					} else {
					ConfigureAccelerometers(AccelMask,ACCEL_STATUS_ENABLED|ACCEL_STATUS_SINGLE_SAMPLE|ACCEL_STATUS_POLLING,0);
					APStatus.flags|=ACCEL_PCB_STATUS_ACCEL_POLLING; //so timer will call GetSample
				}
				//Get_Accelerometer_Samples();

			break;
			} //switch

		} //if (pcb->local_port==UDP_PORT) {
		pbuf_free(p);
	} //if (p != NULL) {
} //void udpserver_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port)


int CheckWired(void) {
	int32_t ret;

	//try to bring up wired network

	eth_ipstack_init();

	//wait for wired network connection
	do {
		ret = ethernet_phy_get_link_status(&ETHERNET_PHY_0_desc, &link_up);
		if (ret == ERR_NONE && link_up) {
			break;
		}
	} while (true);


	printf("Ethernet Connection established\n");
	LWIP_MACIF_init(mac);  //tph: add LWIP callback for recvd input: ethernet_input()

	//make this the default interface
	netif_set_default(&LWIP_MACIF_desc);
	
	// Set callback function for netif status change 
	netif_set_status_callback(&LWIP_MACIF_desc, status_callback);

	//Set callback function for link status change
	netif_set_link_callback(&LWIP_MACIF_desc, link_callback);
		
	mac_async_enable(&ETHERNET_MAC_0);

	udpserver_pcb = udp_new();  //create udp server
	udp_bind(udpserver_pcb, &LWIP_MACIF_desc.ip_addr.addr, UDP_PORT);   //port UDP_PORT 
	udp_recv(udpserver_pcb, udpserver_recv, NULL);  //set udpserver callback function

	//bring up the network interface - ned to do here so above interrupts are enabled
#ifdef LWIP_DHCP
	/* DHCP mode. */
	if (ERR_OK != dhcp_start(&LWIP_MACIF_desc)) {
		LWIP_ASSERT("ERR_OK != dhcp_start", 0);
	}
	printf("DHCP Started\r\n");
#else
	//needed for lwip 2.0: netif_set_link_up(&LWIP_MACIF_desc);
	/* Static mode. */
	netif_set_up(&LWIP_MACIF_desc);
	printf("Static IP Address Assigned\r\n");
#endif

	return(1);

}

int main(void)
{
	struct io_descriptor *io;
	int count;//,StartDHCP;
	uint8_t OutStr[256];
	int32_t ret;
	u8_t ReadBuffer[256];
	//struct usart_async_status iostat;  //currently needed for usart async

	/* Initializes MCU, drivers and middleware - tph - inits phy and uarts*/
	atmel_start_init();

	//initialize user gpio pins	
	//gpio_set_pin_level(LED0,true);
	// Set pin direction to output
	//gpio_set_pin_direction(LED0, GPIO_DIRECTION_OUT);
	//gpio_set_pin_function(LED0, GPIO_PIN_FUNCTION_OFF);

//USART_ASYNC_TXC_CB
	usart_async_register_callback(&USART_1, USART_ASYNC_RXC_CB, usart1_receive_cb);
	usart_async_enable(&USART_1);



	/* Read MacAddress from EEPROM */  //tph: currently just adding a valid public MAC address
	read_macaddress(mac);

	systick_enable();

	//MACIF_example();
	
	ETHERNET_PHY_0_example();  //restarts autonegotiation


	//init usart
	usart_sync_get_io_descriptor(&USART_0, &io);
	usart_sync_enable(&USART_0);
	//usart_async_get_io_descriptor(&USART_0, &io);
	//usart_async_enable(&USART_0);
	count=0;
	sprintf((char *)OutStr,"**************************\n");
	io_write(io,OutStr,strlen(OutStr));
	
	//while (usart_async_get_status(&USART_0, &iostat)==ERR_BUSY);
	//sprintf((char *)OutStr,"\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\n");
	//io_write(io,OutStr,strlen(OutStr));
	sprintf((char *)OutStr,"GAMTP_rev03\n");
	io_write(io,OutStr,strlen(OutStr));
	//while (usart_async_get_status(&USART_0, &iostat)==ERR_BUSY);

	//sprintf((char *)OutStr,"\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\n");
	//io_write(io,OutStr,strlen(OutStr));
	sprintf((char *)OutStr,"**************************\n");
	io_write(io,OutStr,strlen(OutStr));
	//while (usart_async_get_status(&USART_0, &iostat)==ERR_BUSY);

	//sprintf((char *)OutStr,"\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\n");
	//io_write(io,OutStr,strlen(OutStr));



	printf("\r\nHello ATMEL World!\r\n");
	//fflush(stdio_io);

	//below does not work for printf because printf calls _puts_r which must send one char at a time 
	//while (usart_async_get_status(&USART_0, &iostat)==ERR_BUSY); 


	mac_async_register_callback(&ETHERNET_MAC_0, MAC_ASYNC_RECEIVE_CB, (FUNC_PTR)mac_receive_cb);
	mac_async_register_callback(&ETHERNET_MAC_0, MAC_ASYNC_TRANSMIT_CB, (FUNC_PTR)mac_transmit_cb);


	CheckWired();

#if 0 	
	eth_ipstack_init();
	do {
		ret = ethernet_phy_get_link_status(&ETHERNET_PHY_0_desc, &link_up);
		if (ret == ERR_NONE && link_up) {
			break;
		}
	} while (true);
	printf("Ethernet Connection established\n");
	LWIP_MACIF_init(mac);  //tph: add LWIP callback for recvd input: ethernet_input()

	//make this the default interface
	netif_set_default(&LWIP_MACIF_desc);
	
	// Set callback function for netif status change 
	netif_set_status_callback(&LWIP_MACIF_desc, status_callback);

	//Set callback function for link status change
	netif_set_link_callback(&LWIP_MACIF_desc, link_callback);

		
	mac_async_enable(&ETHERNET_MAC_0);

	udpserver_pcb = udp_new();  //create udp server
	udp_bind(udpserver_pcb, &LWIP_MACIF_desc.ip_addr.addr, UDP_PORT);   //port UDP_PORT 
	udp_recv(udpserver_pcb, udpserver_recv, NULL);  //set udpserver callback function


	//bring up the network interface - ned to do here so above interrupts are enabled
#ifdef LWIP_DHCP
	/* DHCP mode. */
	if (ERR_OK != dhcp_start(&LWIP_MACIF_desc)) {
		LWIP_ASSERT("ERR_OK != dhcp_start", 0);
	}
	printf("DHCP Started\r\n");
#else
	//needed for lwip 2.0: netif_set_link_up(&LWIP_MACIF_desc);
	/* Static mode. */
	netif_set_up(&LWIP_MACIF_desc);
	printf("Static IP Address Assigned\r\n");
#endif

#endif

	InitializeMotors(); //set initial settings of all motors
	InitializeAccels(); //set initial settings of all accelerometers
	
	//currently motor timer stop DHCP from working
	MotorTimer_Initialize();  //initialize and start timer for motor pwm
	AccelTimer_Initialize(); //initialize timer for accelerometer sampling
	//Reset all accelerometers
	
	/* Replace with your application code */
	while (true) {

		//if no wired connection yet, check for one



	/* Print IP address info */
	if (!got_ip && link_up && LWIP_MACIF_desc.ip_addr.addr) {
//			link_up = false;
		print_ipaddress();
		got_ip = true;
	}


/*
		if (StartDHCP) {
			StartDHCP=0;
			dhcp_start(&LWIP_MACIF_desc); //tph start dhcp
		}
*/

		if (gmac_recv_flag) {
			//printf("gmac_recd");
			//sprintf((char *)OutStr,"recvd2\n");
			//io_write(io,OutStr,strlen(OutStr));
			
			gmac_recv_flag = false;
			ethernetif_mac_input(&LWIP_MACIF_desc);
		}
		/* LWIP timers - ARP, DHCP, TCP, etc. */
		sys_check_timeouts();


		//netif_poll(&LWIP_MACIF_desc);  //tph need?

		//check interface for DHCP
//		if (LWIP_MACIF_desc.dhcp->state == DHCP_BOUND) {
//			sprintf((char *)OutStr,"DHCP bound\n");
//			io_write(io,OutStr,strlen(OutStr));			
//		}
	//autoip_tmr(); //call every 100ms AUTOIP_TMR_INTERVAL msces,

/*		delay_ms(1000);
		gpio_toggle_pin_level(LED0);
		
		sprintf((char *)OutStr,"\b\b\b%03d",count);
		io_write(io, OutStr, strlen(OutStr));
		count++;
		//USART_0_example();
*/

//	GMAC_Handler();
	//mac_async_read(&MACIF, ReadBuffer, 10);

#if 0 
	volatile uint32_t imr,isr,ier,ncr,ncfgr,ur,rsr,dcfgr,nsr,tsr;
	//read GMAC interrupt mask register to confirm which interrupts are enabled (=0, RCOMP: receive complete= bit1)
	imr=hri_gmac_read_IMR_reg(GMAC);  //interrupt mask register
	isr=hri_gmac_read_ISR_reg(GMAC);  //interrupt status register
	//ier=hri_gmac_read_IER_reg(GMAC);  //interrupt enabled register
	ncr=hri_gmac_read_NCR_reg(GMAC);  //network control register
	ncfgr=hri_gmac_read_NCFGR_reg(GMAC);  //network configuration register
	ur=hri_gmac_read_UR_reg(GMAC);  //user register - bit 0=0 for RMII
	dcfgr=hri_gmac_read_DCFGR_reg(GMAC);  //DMA Configuration register 
	rsr=hri_gmac_read_RSR_reg(GMAC);  //user register - bit 0=0 for RMII
	nsr=hri_gmac_read_NSR_reg(GMAC);  //bit 1 and 2
	tsr=hri_gmac_read_TSR_reg(GMAC);  //bit 5 tx complete
#endif	
	//could test loop back send and receive: set LBL bit in NCR


	USART_1_input();  //check for usart1 input

	//delay_ms(100);
	
	

	}  //while(1)

}
