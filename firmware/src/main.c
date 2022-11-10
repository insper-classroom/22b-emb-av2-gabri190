#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

//input analogico
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

#define PIO_PWM_0 PIOD
#define ID_PIO_PWM_0 ID_PIOD
#define MASK_PIN_PWM_0 (1 << 11)

#define PIO_PWM_1 PIOA
#define ID_PIO_PWM_1 ID_PIOA
#define MASK_PIN_PWM_1 (1 << 6)


#define PIO_PWM_2 PIOD
#define ID_PIO_PWM_2 ID_PIOD
#define MASK_PIN_PWM_2 (1 << 26)

QueueHandle_t xQueueAFEC;
QueueHandle_t xQueueRGB;

typedef struct {
	uint value;
} adcData;

typedef struct {
	uint *r;
	uint *g;
	uint *b;
} rgb;

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

/** RTOS  */
#define TASK_AFEC_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_AFEC_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void PWM_init(Pwm *p_pwm, uint id_pwm, pwm_channel_t *p_channel, uint channel, uint duty);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback) ;
void wheel( uint WheelPos, uint *r, uint *g, uint *b );

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);
}

static void AFEC_pot_Callback(void) {
	adcData adc;
	adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueAFEC, &adc, &xHigherPriorityTaskWoken);
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/


static void task_afec(void *pvParameters){
		int afec_flag;
		/* inicializa e configura adc */
		config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
		adcData adc;
		rgb color;
		RTT_init(10, 1000, RTT_MR_ALMIEN);
		/* Selecina canal e inicializa conversão */
		afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
		afec_start_software_conversion(AFEC_POT);
		int position;
		while(1){
			if(xQueueReceive(xQueueAFEC,&(afec_flag),(TickType_t) 0)){
				printf("recebendo %d\n",afec_flag);
				if(color.r==1 && color.g==0 && color.b==0){
						wheel(position,255-3*position,0,position*3);
						xQueueSend(xQueueRGB,(void *)&color.r,(TickType_t)10);
				}else if(color.r==0 && color.g==1 && color.b==0){
						wheel(position,0,3*position,255-position*3);
						xQueueSend(xQueueRGB,(void *)&color.g,(TickType_t)10);
				}else if(color.r==0 && color.g==0 && color.b==1){
						wheel(position,3*position,255-3*position,0);
						xQueueSend(xQueueRGB,(void *)&color.b,(TickType_t)10);
				}
				
		}

	}
		
}
static void task_led(void *pvParameters) {
	 rgb color;
	 pmc_enable_periph_clk(ID_PIOD);
	 pio_set_peripheral(PIOD, PIO_PERIPH_B, 1 << 12);

	 static pwm_channel_t pwm_channel_pd11;
	 PWM_init(PWM0, ID_PWM0,  &pwm_channel_pd11, PWM_CHANNEL_0, 0);
	 
	pmc_enable_periph_clk(ID_PIOA);
	pio_set_peripheral(PIOA, PIO_PERIPH_C, 1 << 12);

	static pwm_channel_t pwm_channel_pa12;
	PWM_init(PWM1, ID_PWM1,  &pwm_channel_pa12, PWM_CHANNEL_0, 0);
	 	
	 /* duty cycle */
	 int duty = 0;
	 if(xQueueReceive(xQueueRGB,&(color.r),(TickType_t) 0)){
		  /* fade in */
		  for(duty = 0; duty <= 255; duty++){
			  pwm_channel_update_duty(PWM0, &pwm_channel_pd11, 255-duty);
			  delay_ms(10);
		  }
		  /* fade out*/
		  for(duty = 0; duty <= 255; duty++){
			  pwm_channel_update_duty(PWM0, &pwm_channel_pd11, duty);
			  delay_ms(10);
		  }
	 }
	 else if(xQueueReceive(xQueueRGB,&(color.g),(TickType_t) 0)){
		/* fade in */
		for(duty = 0; duty <= 255; duty++){
			pwm_channel_update_duty(PWM1, &pwm_channel_pa12, 255-duty);
			delay_ms(10);
		}
		/* fade out*/
		for(duty = 0; duty <= 255; duty++){
			pwm_channel_update_duty(PWM1, &pwm_channel_pa12, duty);
			delay_ms(10);
		}
		 
	 }
	 /* Infinite loop */
	 while (1) {
		
	 }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void wheel( uint WheelPos, uint *r, uint *g, uint *b ) {
	rgb color;
	WheelPos = 255 - WheelPos;
	if(WheelPos<85){
		color.r=255-WheelPos*3;
		color.g=0;
		color.b=WheelPos*3;
	}
	else if(WheelPos<170){
		WheelPos-=85;
		color.r=0;
		color.g=WheelPos*3;
		color.b=255-WheelPos*3;
		
	}else{
		WheelPos-=170;
		color.r=WheelPos*3;
		color.g=255-WheelPos*3;
		color.b=0;
	}
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
afec_callback_t callback) {
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
	afec_enable(afec);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(afec, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(afec, AFEC_TRIG_SW);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(afec, afec_channel, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

	/* configura IRQ */
	afec_set_callback(afec, afec_channel, callback, 1);
	NVIC_SetPriority(afec_id, 4);
	NVIC_EnableIRQ(afec_id);
}


static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses,
uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT))
		;
		rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

void PWM_init(Pwm *p_pwm, uint id_pwm, pwm_channel_t *p_channel, uint channel, uint duty){
	
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(id_pwm);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(p_pwm, channel);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = 1000 * 256,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(p_pwm, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	p_channel->alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	p_channel->polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	p_channel->ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	p_channel->ul_period = 256;
	/* Duty cycle value of output waveform */
	p_channel->ul_duty = duty;
	p_channel->channel = channel;
	pwm_channel_init(p_pwm, p_channel);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(p_pwm, channel);
}



/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_led, "led", TASK_OLED_STACK_SIZE, NULL,
	TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create task_led\r\n");
	}
	/* Create task to control oled */
	if (xTaskCreate(task_led, "led", TASK_OLED_STACK_SIZE, NULL,
	TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create task_led\r\n");
	}
	xQueueAFEC = xQueueCreate(100, sizeof(unsigned long));
	if (xQueueAFEC == NULL)
	printf("falha em criar a queue xQueueLedBluet \n");
	
	xQueueRGB = xQueueCreate(100, sizeof(unsigned long));
	if (xQueueRGB == NULL)
	printf("falha em criar a queue xQueueBuzBluet \n");
	
	/* Create task to make led blink */
	if(xTaskCreate(task_afec, "AFEC", TASK_AFEC_STACK_SIZE, NULL,	TASK_OLED_STACK_PRIORITY, NULL)!=pdPASS){
		printf("Failed to create BLT task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS n?o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}