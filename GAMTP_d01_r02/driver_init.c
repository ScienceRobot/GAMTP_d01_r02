/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

/*! The buffer size for USART */
#define USART_1_BUFFER_SIZE 256//tph 16

struct usart_async_descriptor USART_1;
struct timer_descriptor       TIMER_0;

static uint8_t USART_1_buffer[USART_1_BUFFER_SIZE];

struct adc_sync_descriptor ADC_0;

struct adc_sync_descriptor ADC_1;

struct usart_sync_descriptor USART_0;

struct i2c_m_sync_desc I2C_0;

struct mac_async_descriptor ETHERNET_MAC_0;

void ADC_0_PORT_init(void)
{

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA02, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA02, PINMUX_PA02B_ADC0_AIN0);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA03, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA03, PINMUX_PA03B_ADC0_AIN1);
}

void ADC_0_CLOCK_init(void)
{
	hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void ADC_0_init(void)
{
	ADC_0_CLOCK_init();
	ADC_0_PORT_init();
	adc_sync_init(&ADC_0, ADC0, (void *)NULL);
}

void ADC_1_PORT_init(void)
{

	// Disable digital pin circuitry
	gpio_set_pin_direction(PC02, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PC02, PINMUX_PC02B_ADC1_AIN4);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PC03, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PC03, PINMUX_PC03B_ADC1_AIN5);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PB04, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PB04, PINMUX_PB04B_ADC1_AIN6);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PB05, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PB05, PINMUX_PB05B_ADC1_AIN7);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PC00, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PC00, PINMUX_PC00B_ADC1_AIN10);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PC01, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PC01, PINMUX_PC01B_ADC1_AIN11);
}

void ADC_1_CLOCK_init(void)
{
	hri_mclk_set_APBDMASK_ADC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC1_GCLK_ID, CONF_GCLK_ADC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void ADC_1_init(void)
{
	ADC_1_CLOCK_init();
	ADC_1_PORT_init();
	adc_sync_init(&ADC_1, ADC1, (void *)NULL);
}

void USART_0_PORT_init(void)
{

	gpio_set_pin_function(PC27, PINMUX_PC27C_SERCOM1_PAD0);

	gpio_set_pin_function(PC28, PINMUX_PC28C_SERCOM1_PAD1);
}

void USART_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBAMASK_SERCOM1_bit(MCLK);
}

void USART_0_init(void)
{
	USART_0_CLOCK_init();
	usart_sync_init(&USART_0, SERCOM1, (void *)NULL);
	USART_0_PORT_init();
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(PA22,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA22, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_pull_mode(PA23,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA23, PINMUX_PA23C_SERCOM3_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBBMASK_SERCOM3_bit(MCLK);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM3);
	I2C_0_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_1_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM6_GCLK_ID_CORE, CONF_GCLK_SERCOM6_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM6_GCLK_ID_SLOW, CONF_GCLK_SERCOM6_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBDMASK_SERCOM6_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_1_PORT_init()
{

	gpio_set_pin_function(PC16, PINMUX_PC16C_SERCOM6_PAD0);

	gpio_set_pin_function(PC17, PINMUX_PC17C_SERCOM6_PAD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_1_init(void)
{
	USART_1_CLOCK_init();
	usart_async_init(&USART_1, SERCOM6, USART_1_buffer, USART_1_BUFFER_SIZE, (void *)NULL);
	USART_1_PORT_init();
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	hri_mclk_set_APBAMASK_TC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	timer_init(&TIMER_0, TC0, _tc_get_timer());
}

void ETHERNET_MAC_0_PORT_init(void)
{

	gpio_set_pin_function(PC11, PINMUX_PC11L_GMAC_GMDC);

	gpio_set_pin_function(PC12, PINMUX_PC12L_GMAC_GMDIO);

	gpio_set_pin_function(PA13, PINMUX_PA13L_GMAC_GRX0);

	gpio_set_pin_function(PA12, PINMUX_PA12L_GMAC_GRX1);

	gpio_set_pin_function(PC20, PINMUX_PC20L_GMAC_GRXDV);

	gpio_set_pin_function(PA15, PINMUX_PA15L_GMAC_GRXER);

	gpio_set_pin_function(PA18, PINMUX_PA18L_GMAC_GTX0);

	gpio_set_pin_function(PA19, PINMUX_PA19L_GMAC_GTX1);

	gpio_set_pin_function(PA14, PINMUX_PA14L_GMAC_GTXCK);

	gpio_set_pin_function(PA17, PINMUX_PA17L_GMAC_GTXEN);
}

void ETHERNET_MAC_0_CLOCK_init(void)
{
	hri_mclk_set_AHBMASK_GMAC_bit(MCLK);
	hri_mclk_set_APBCMASK_GMAC_bit(MCLK);
}

void ETHERNET_MAC_0_init(void)
{
	ETHERNET_MAC_0_CLOCK_init();
	mac_async_init(&ETHERNET_MAC_0, GMAC);
	ETHERNET_MAC_0_PORT_init();
}

void ETHERNET_MAC_0_example(void)
{
	mac_async_enable(&ETHERNET_MAC_0);
	mac_async_write(&ETHERNET_MAC_0, (uint8_t *)"Hello World!", 12);
}

void system_init(void)
{
	init_mcu();

	ADC_0_init();

	ADC_1_init();

	USART_0_init();

	I2C_0_init();
	USART_1_init();

	TIMER_0_init();
	ETHERNET_MAC_0_init();
}
