

#include <asf.h>

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

// ADC
struct adc_module adc_instance;

// USART
struct usart_module usart_instance;
#define MAX_RX_BUFFER_LENGTH   5					
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];	// Not used
volatile unsigned int sys_timer1 = 0;

/************************************************************************/
/* Function Prototypes                                                  */
/************************************************************************/

void usart_read_callback(const struct usart_module *const usart_module);
void usart_write_callback(const struct usart_module *const usart_module);

/************************************************************************/
/* INIT Clocks                                                          */
/************************************************************************/

void enable_tc_clocks(void)
{
	
	struct system_gclk_chan_config gclk_chan_conf;
	
	/* Turn on TC module in PM */
	system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBC, PM_APBCMASK_TC3);

	/* Set up the GCLK for the module */
	system_gclk_chan_get_config_defaults(&gclk_chan_conf);
	
	//Setup generic clock 0 (also the clock for MCU (running at 8 Mhz) as source for the timer clock)
	gclk_chan_conf.source_generator = GCLK_GENERATOR_0;
	system_gclk_chan_set_config(TC3_GCLK_ID, &gclk_chan_conf);
	
	//Enable the generic clock for the Timer/ Counter block
	system_gclk_chan_enable(TC3_GCLK_ID);
}

/************************************************************************/
/* INIT ADC                                                             */
/************************************************************************/

void configure_adc(void)
{
	struct adc_config config_adc;
	
	// Pre configure with defaults 
	adc_get_config_defaults(&config_adc);
	
	// Customize
	config_adc.gain_factor     = ADC_GAIN_FACTOR_DIV2;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
	config_adc.reference       = ADC_REFERENCE_INTVCC1;
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN8;	// PB00 on ext1
	config_adc.resolution      = ADC_RESOLUTION_12BIT;
	
	// INIT and Enable
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}

/************************************************************************/
/* INIT USART                                                           */
/************************************************************************/

void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	while (usart_init(&usart_instance,
	EDBG_CDC_MODULE, &config_usart) != STATUS_OK) {
	}
	usart_enable(&usart_instance);
}

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,
	usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
	usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

int main (void)
{
	system_init();
	system_clock_init();
	
	enable_tc_clocks();
	SysTick_Config(8000);
	
	configure_adc();
	
	configure_usart();
	configure_usart_callbacks();
	
	uint8_t string[] = "This is the SAMD20 with your ADC readings!\r\n";
	usart_write_buffer_job(&usart_instance, string, sizeof(string));
	 
	while (1)
	{
		if(sys_timer1 > 1000)
		{
			adc_start_conversion(&adc_instance);
			uint16_t result;
			do 
			{
				// Wait for ADC conversion
			} while (adc_read(&adc_instance, &result) == STATUS_BUSY);
			
			// Convert reading to a string and write the buffer
			char tx_buffer[80];
			sprintf(tx_buffer, "ADC Reading = %u\r\n", result);
			usart_write_buffer_job(&usart_instance, tx_buffer, sizeof(tx_buffer));
			
			// Reset sys_timer1
			sys_timer1 = 0;	
		}
	}
}

/************************************************************************/
/* Subroutines                                                          */
/************************************************************************/

void usart_read_callback(const struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_instance,
	(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
}

void usart_write_callback(const struct usart_module *const usart_module)
{
	port_pin_toggle_output_level(LED_0_PIN);
}

void SysTick_Handler(void)
{
	sys_timer1++;
}