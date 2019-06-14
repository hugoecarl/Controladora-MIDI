#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// TRIGGER
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// usart (bluetooth)
#define USART_COM_ID ID_USART0
#define USART_COM    USART0

#define APERTADO '1'
#define LIBERADO '0'


#define EBUT2_PIO PIOD //start EXT 9 PD28
#define EBUT2_PIO_ID ID_PIOD
#define EBUT2_PIO_IDX 16
#define EBUT2_PIO_IDX_MASK (1u << EBUT2_PIO_IDX)

#define EBUT1_PIO PIOC //pause  Ext 4 PA19 PA = 10
#define EBUT1_PIO_ID ID_PIOC
#define EBUT1_PIO_IDX 9
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)

#define LED1_PIO PIOD
#define LED1_PIO_ID ID_PIOD
#define LED1_PIO_IDX 30
#define LED1_PIO_IDX_MASK (1u << LED1_PIO_IDX)

#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 19
#define LED2_PIO_IDX_MASK (1u << LED2_PIO_IDX)

#define LED3_PIO PIOD
#define LED3_PIO_ID ID_PIOD
#define LED3_PIO_IDX 11
#define LED3_PIO_IDX_MASK (1u << LED3_PIO_IDX)

#define LED4_PIO PIOC
#define LED4_PIO_ID ID_PIOC
#define LED4_PIO_IDX 13
#define LED4_PIO_IDX_MASK (1u << LED4_PIO_IDX)

#define LED5_PIO PIOD
#define LED5_PIO_ID ID_PIOD
#define LED5_PIO_IDX 26
#define LED5_PIO_IDX_MASK (1u << LED5_PIO_IDX)

#define LED6_PIO PIOA
#define LED6_PIO_ID ID_PIOA
#define LED6_PIO_IDX 21
#define LED6_PIO_IDX_MASK (1u << LED6_PIO_IDX)

#define LED7_PIO PIOA
#define LED7_PIO_ID ID_PIOA
#define LED7_PIO_IDX 6
#define LED7_PIO_IDX_MASK (1u << LED7_PIO_IDX)

#define LED8_PIO PIOA
#define LED8_PIO_ID ID_PIOA
#define LED8_PIO_IDX 24
#define LED8_PIO_IDX_MASK (1u << LED8_PIO_IDX)

#define LED9_PIO PIOB
#define LED9_PIO_ID ID_PIOB
#define LED9_PIO_IDX 4
#define LED9_PIO_IDX_MASK (1u << LED9_PIO_IDX)

#define LED10_PIO PIOA
#define LED10_PIO_ID ID_PIOA
#define LED10_PIO_IDX 26
#define LED10_PIO_IDX_MASK (1u << LED10_PIO_IDX)


#define LED11_PIO PIOA
#define LED11_PIO_ID ID_PIOA
#define LED11_PIO_IDX 2
#define LED11_PIO_IDX_MASK (1u << LED11_PIO_IDX)

#define LED12_PIO PIOD
#define LED12_PIO_ID ID_PIOD
#define LED12_PIO_IDX 27
#define LED12_PIO_IDX_MASK (1u << LED12_PIO_IDX)

#define BUZZ_PIO           PIOA                  // periferico que controla o LED
#define BUZZ_PIO_ID        ID_PIOA                   // ID do periférico PIOC (controla LED)
#define BUZZ_PIO_IDX      12u                    // ID do LED no PIO
#define BUZZ_PIO_IDX_MASK  (1u << BUZZ_PIO_IDX)   // Mascara para CONTROLARMOS o LED


/** UART Interface */
#define CONF_UART              CONSOLE_UART
/** Baudrate setting */
#define CONF_UART_BAUDRATE     (115200UL)
/** Character length setting */
#define CONF_UART_CHAR_LENGTH  US_MR_CHRL_8_BIT
/** Parity setting */
#define CONF_UART_PARITY       US_MR_PAR_NO
/** Stop bits setting */
#define CONF_UART_STOP_BITS    US_MR_NBSTOP_1_BIT

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)

#define AFEC_CHANNEL_POT_SENSOR 8

/** RTOS  */

#define TASK_SLIDER_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_SLIDER_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_BOTOES_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_BOTOES_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_PROCESS_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_PROCESS_STACK_PRIORITY        (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


/** prototypes */
void but_callback(void);
static void ECHO_init(void);
static void USART1_init(void);
uint32_t usart_puts(uint8_t *pstring);


QueueHandle_t xQueue1;
volatile uint32_t g_tcCv = 0;
volatile Bool but_status;
volatile Bool g1_is_conversion_done = false;
volatile uint32_t pot_ul_value;
char buffer[6];
char bufferx[6];
QueueHandle_t xQueuePot;

typedef void (*p_handler) (uint32_t, uint32_t);

//Define e inicia struct para botoes

typedef bitset<8> BYTE;

typedef struct {
	uint32_t PIO_NAME;
	uint32_t PIO_ID;
	uint32_t PIO_IDX;
	uint32_t PIO_MASK;
	BYTE data;
	uint32_t data1;
	uint32_t data2;
	volatile Bool but_flag;
	SemaphoreHandle_t xSemaphore;
	char BUT_NUM;
} botao;

botao BUT3 = {.PIO_NAME = PIOD, .PIO_ID = ID_PIOD, .PIO_IDX = 28u, .PIO_MASK = (1u << 28u), .BUT_NUM = 'a', .data =0x90, .data1 = 1, .data2 = 100};
botao BUT7 = {.PIO_NAME = PIOB, .PIO_ID = ID_PIOB, .PIO_IDX = 2u, .PIO_MASK = (1u << 2u), .BUT_NUM = 'b', .data = 0x90, .data1 = 2, .data2 = 100};
botao BUT9 = {.PIO_NAME = PIOC, .PIO_ID = ID_PIOC, .PIO_IDX = 30u, .PIO_MASK = (1u << 30u), .BUT_NUM = 'c', .data = 0x90, .data1 = 3, .data2 = 100};
botao BUT8 = {.PIO_NAME = PIOD, .PIO_ID = ID_PIOD, .PIO_IDX = 22u, .PIO_MASK = (1u << 22u), .BUT_NUM = 'd', .data = 0x90, .data1 = 4, .data2 = 100};
botao BUT11 = {.PIO_NAME = PIOD, .PIO_ID = ID_PIOD, .PIO_IDX = 21u, .PIO_MASK = (1u << 21u), .BUT_NUM = 'e', .data = 0x90, .data1 = 5, .data2 = 100};
botao BUT2 = {.PIO_NAME = PIOA, .PIO_ID = ID_PIOA, .PIO_IDX = 4u, .PIO_MASK = (1u << 4u), .BUT_NUM = 'f', .data = 0x90, .data1 = 6, .data2 = 100};
botao BUT12 = {.PIO_NAME = PIOA, .PIO_ID = ID_PIOA, .PIO_IDX = 3u, .PIO_MASK = (1u << 3u), .BUT_NUM = 'g', .data = 0x90, .data1 = 7, .data2 = 100};
botao BUT6 = {.PIO_NAME = PIOB, .PIO_ID = ID_PIOB, .PIO_IDX = 3u, .PIO_MASK = (1u << 3u), .BUT_NUM = 'h', .data = 0x90, .data1 = 8, .data2 = 100};
botao BUT5 = {.PIO_NAME = PIOD, .PIO_ID = ID_PIOD, .PIO_IDX = 25u, .PIO_MASK = (1u << 25u), .BUT_NUM = 'i', .data = 0x90, .data1 = 9, .data2 = 100};
botao BUT1 = {.PIO_NAME = PIOA, .PIO_ID = ID_PIOA, .PIO_IDX = 0u, .PIO_MASK = (1u << 0u), .BUT_NUM = 'j', .data = 0x90, .data1 = 10, .data2 = 100};
botao BUT10 = {.PIO_NAME = PIOC, .PIO_ID = ID_PIOC, .PIO_IDX = 17u, .PIO_MASK = (1u << 17u), .BUT_NUM = 'k', .data = 0x90, .data1 = 11, .data2 = 100};
botao BUT4 = {.PIO_NAME = PIOD, .PIO_ID = ID_PIOD, .PIO_IDX = 20u, .PIO_MASK = (1u << 20u), .BUT_NUM = 'l', .data = 0x90, .data1 = 12, .data2 = 100};

//botao *o[12] = {&BUT1, &BUT2, &BUT3, &BUT4, &BUT5, &BUT6, &BUT7, &BUT8, &BUT9, &BUT10, &BUT11, &BUT12};



static void AFEC_pot_callback(void)
{
	pot_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT_SENSOR);
	g1_is_conversion_done = true;
	xQueueSendFromISR( xQueuePot, &pot_ul_value, 0);
}

void but1_callback(void)
{
	if(!pio_get(BUT1.PIO_NAME, PIO_INPUT, BUT1.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT1.xSemaphore, NULL);
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		}else{
		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
	}
}

void but2_callback(void)
{
	if(!pio_get(BUT2.PIO_NAME, PIO_INPUT, BUT2.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT2.xSemaphore, NULL);
		pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
		}else{
		pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
	}
}

void but3_callback(void)
{
	if(!pio_get(BUT3.PIO_NAME, PIO_INPUT, BUT3.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT3.xSemaphore, NULL);
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
		}else{
		pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
	}
}

void but4_callback(void)
{
	if(!pio_get(BUT4.PIO_NAME, PIO_INPUT, BUT4.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT4.xSemaphore, NULL);
		pio_set(LED4_PIO, LED4_PIO_IDX_MASK);
		}else{
		pio_clear(LED4_PIO, LED4_PIO_IDX_MASK);
	}
}

void but5_callback(void)
{
	if(!pio_get(BUT5.PIO_NAME, PIO_INPUT, BUT5.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT5.xSemaphore, NULL);
		pio_set(LED5_PIO, LED5_PIO_IDX_MASK);
		}else{
		pio_clear(LED5_PIO, LED5_PIO_IDX_MASK);
	}
}

void but6_callback(void)
{
	if(!pio_get(BUT6.PIO_NAME, PIO_INPUT, BUT6.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT6.xSemaphore, NULL);
		pio_set(LED6_PIO, LED6_PIO_IDX_MASK);
		}else{
		pio_clear(LED6_PIO, LED6_PIO_IDX_MASK);
	}
}


void but7_callback(void)
{
	if(!pio_get(BUT7.PIO_NAME, PIO_INPUT, BUT7.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT7.xSemaphore, NULL);
		pio_set(LED7_PIO, LED7_PIO_IDX_MASK);
		}else{
		pio_clear(LED7_PIO, LED7_PIO_IDX_MASK);
	}
}

void but8_callback(void)
{
	if(!pio_get(BUT8.PIO_NAME, PIO_INPUT, BUT8.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT8.xSemaphore, NULL);
		pio_set(LED8_PIO, LED8_PIO_IDX_MASK);
		}else{
		pio_clear(LED8_PIO, LED8_PIO_IDX_MASK);
	}
}

void but9_callback(void)
{
	if(!pio_get(BUT9.PIO_NAME, PIO_INPUT, BUT9.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT9.xSemaphore, NULL);
		pio_set(LED9_PIO, LED9_PIO_IDX_MASK);
		}else{
		pio_clear(LED9_PIO, LED9_PIO_IDX_MASK);
	}
}

void but10_callback(void)
{
	if(!pio_get(BUT10.PIO_NAME, PIO_INPUT, BUT10.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT10.xSemaphore, NULL);
		pio_set(LED10_PIO, LED10_PIO_IDX_MASK);
		}else{
		pio_clear(LED10_PIO, LED10_PIO_IDX_MASK);
	}
}

void but11_callback(void)
{
	if(!pio_get(BUT11.PIO_NAME, PIO_INPUT, BUT11.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT11.xSemaphore, NULL);
		pio_set(LED11_PIO, LED11_PIO_IDX_MASK);
		}else{
		pio_clear(LED11_PIO, LED11_PIO_IDX_MASK);
	}
}

void but12_callback(void)
{
	if(!pio_get(BUT12.PIO_NAME, PIO_INPUT, BUT12.PIO_MASK)){
		xSemaphoreGiveFromISR(BUT12.xSemaphore, NULL);
		pio_set(LED12_PIO, LED12_PIO_IDX_MASK);
		}else{
		pio_clear(LED12_PIO, LED12_PIO_IDX_MASK);
	}
}

void bizz(int frequencia, int tempo_ms, int w_led){
	int us_delay = 1000000/frequencia; //quantos microsegundos entre as notas para definir a frequencia
	int tempo = tempo_ms * 1000; //ms -> us
	
	int i = 0;
	
	while (i<tempo/us_delay){
		pio_set(BUZZ_PIO, BUZZ_PIO_IDX_MASK);      // Coloca 1 no pino LED
		delay_us(us_delay/2);
		pio_clear(BUZZ_PIO, BUZZ_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		delay_us(us_delay/2);
		i++;
	}
	
	
	
}



/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure the console UART.
 */

static void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

uint32_t usart_puts(uint8_t *pstring){
	uint32_t i ;

	while(*(pstring + i))
		if(uart_is_tx_empty(USART_COM))
			usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
  usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
  uint timecounter = timeout_ms;
  uint32_t rx;
  uint32_t counter = 0;
  
  while( (timecounter > 0) && (counter < bufferlen - 1)) {
    if(usart_read(usart, &rx) == 0) {
      buffer[counter++] = rx;
    }
    else{
      timecounter--;
      vTaskDelay(1);
    }    
  }
  buffer[counter] = 0x00;
  return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
  usart_put_string(usart, buffer_tx);
  usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif


void send_command(uint32_t nbotao, char buffer[]){
	char eof = 'X';
	buffer[4] = 'x';
	buffer[5] = nbotao;
	while(!usart_is_tx_ready(UART_COMM));
	usart_serial_write_packet(USART0, buffer , 6);
	//usart_write(uart_comm, nbotao);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, eof);
}

void hc05_config_server(void) {
  sysclk_enable_peripheral_clock(USART_COM_ID);
  usart_serial_options_t config;
  config.baudrate = 9600;
  config.charlength = US_MR_CHRL_8_BIT;
  config.paritytype = US_MR_PAR_NO;
  config.stopbits = false;
  usart_serial_init(USART_COM, &config);
  usart_enable_tx(USART_COM);
  usart_enable_rx(USART_COM);
  
  // RX - PB0  TX - PB1
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
  char buffer_rx[128];
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100); printf("AT\n");
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEHUGO", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT+PIN4567", 100);
}



static void config_POT(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_8,	AFEC_pot_callback, 5);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_POT_SENSOR, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_POT_SENSOR, 0x200);

	/***  Configura sensor de temperatura ***/

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT_SENSOR);

	pmc_enable_periph_clk(EBUT1_PIO_ID);
	pio_configure(EBUT1_PIO, PIO_OUTPUT_0, EBUT1_PIO_IDX_MASK, PIO_DEFAULT);
	pmc_enable_periph_clk(EBUT2_PIO_ID);
	pio_configure(EBUT2_PIO, PIO_OUTPUT_0, EBUT2_PIO_IDX_MASK, PIO_DEFAULT);
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT_SENSOR);

}

void iniciabots(botao BOT, p_handler *funcao){
	
	pio_configure(BOT.PIO_NAME, PIO_INPUT, BOT.PIO_MASK, PIO_PULLUP|PIO_DEBOUNCE);
	pio_set_debounce_filter(BOT.PIO_NAME, BOT.PIO_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BOT.PIO_NAME,
	BOT.PIO_ID,
	BOT.PIO_MASK,
	PIO_IT_EDGE,
	funcao);

	// Ativa interrupção
	pio_enable_interrupt(BOT.PIO_NAME, BOT.PIO_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BOT.PIO_ID);
	NVIC_SetPriority(BOT.PIO_ID, 4); // Prioridade 4

}

void led(void) {
	//pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED4_PIO, LED4_PIO_IDX_MASK);
	////delay_ms(300);
	//pio_clear(LED4_PIO, LED4_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED5_PIO, LED5_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED5_PIO, LED5_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED6_PIO, LED6_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED6_PIO, LED6_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED7_PIO, LED7_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED7_PIO, LED7_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED8_PIO, LED8_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED8_PIO, LED8_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED9_PIO, LED9_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED9_PIO, LED9_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED10_PIO, LED10_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED10_PIO, LED10_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED11_PIO, LED11_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED11_PIO, LED11_PIO_IDX_MASK);
	////delay_ms(10);
	//pio_set(LED12_PIO, LED12_PIO_IDX_MASK);
	//delay_ms(300);
	//pio_clear(LED12_PIO, LED12_PIO_IDX_MASK);
	////delay_ms(10);

	pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED4_PIO, LED4_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED5_PIO, LED5_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED6_PIO, LED6_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED7_PIO, LED7_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED8_PIO, LED8_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED9_PIO, LED9_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED10_PIO, LED10_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED11_PIO, LED11_PIO_IDX_MASK);
	delay_ms(80);
	pio_set(LED12_PIO, LED12_PIO_IDX_MASK);
	delay_ms(80);
	pio_clear(LED12_PIO, LED12_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED11_PIO, LED11_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED10_PIO, LED10_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED9_PIO, LED9_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED8_PIO, LED8_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED7_PIO, LED7_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED6_PIO, LED6_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED5_PIO, LED5_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED4_PIO, LED4_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
	delay_ms(30);
	pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
	
	






}

void io_init(void){

	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pmc_enable_periph_clk(LED4_PIO_ID);
	pmc_enable_periph_clk(LED5_PIO_ID);
	pmc_enable_periph_clk(LED6_PIO_ID);
	pmc_enable_periph_clk(LED7_PIO_ID);
	pmc_enable_periph_clk(LED8_PIO_ID);
	pmc_enable_periph_clk(LED9_PIO_ID);
	pmc_enable_periph_clk(LED10_PIO_ID);
	pmc_enable_periph_clk(LED11_PIO_ID);
	pmc_enable_periph_clk(LED12_PIO_ID);
        pmc_enable_periph_clk(BUZZ_PIO_ID)
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED4_PIO, PIO_OUTPUT_0, LED4_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED5_PIO, PIO_OUTPUT_0, LED5_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED6_PIO, PIO_OUTPUT_0, LED6_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED7_PIO, PIO_OUTPUT_0, LED7_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED8_PIO, PIO_OUTPUT_0, LED8_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED9_PIO, PIO_OUTPUT_0, LED9_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED10_PIO, PIO_OUTPUT_0, LED10_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED11_PIO, PIO_OUTPUT_0, LED11_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED12_PIO, PIO_OUTPUT_0, LED12_PIO_IDX_MASK, PIO_DEFAULT);

        pio_configure(BUZZ_PIO, PIO_OUTPUT_0, BUZZ_PIO_IDX_MASK, PIO_DEFAULT);


	
	
	
	
	
	
	iniciabots(BUT1, but1_callback);
	iniciabots(BUT2, but2_callback);
	iniciabots(BUT3, but3_callback);
	iniciabots(BUT4, but4_callback);
	iniciabots(BUT5, but5_callback);
	iniciabots(BUT6, but6_callback);
	iniciabots(BUT7, but7_callback);
	iniciabots(BUT8, but8_callback);
	iniciabots(BUT9, but9_callback);
	iniciabots(BUT10, but10_callback);
	iniciabots(BUT11, but11_callback);
	iniciabots(BUT12, but12_callback);



}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_bluetooth(void){
  
  printf("Bluetooth initializing \n");
  hc05_config_server();
  hc05_server_init();
  io_init();
  led();	
  
  while(1){
    //printf("done\n");
    //vTaskDelay( 500 / portTICK_PERIOD_MS);
  }
}

void task_slider(void){
	xQueuePot = xQueueCreate( 10, sizeof( int32_t ) );
	
	config_POT();
	afec_start_software_conversion(AFEC0);
	
	while(1){
		if (xQueueReceive( xQueuePot, &(pot_ul_value), ( TickType_t )  1 / portTICK_PERIOD_MS)) {
		printf("Inicio conversão \n");
		afec_start_software_conversion(AFEC0);
		sprintf(buffer, "%04d",pot_ul_value);
		send_command('n', buffer);
		//vTaskDelay(1000);
		}
		
	if(usart_get_string(USART0, &bufferx, 6, 100)){
		int n = atoi(bufferx);
		double m = (double) n;
		m *= 40.95;
		
		if (m < pot_ul_value){
			while(m < pot_ul_value && pot_ul_value > 30){
				afec_start_software_conversion(AFEC0);
				pio_set(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
				pio_clear(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
				delay_ms(1);
				pio_clear(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
				pio_clear(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
				delay_us(300);
			}
			} else {
			while(m > pot_ul_value && pot_ul_value < 4094){
				afec_start_software_conversion(AFEC0);
				pio_clear(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
				pio_set(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
				delay_ms(1);
				pio_clear(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
				pio_clear(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
				delay_us(300);
			}
		}
	}
	}
}
	
void task_botoes(void){
	
	BUT1.xSemaphore = xSemaphoreCreateBinary();
	BUT2.xSemaphore = xSemaphoreCreateBinary();
	BUT3.xSemaphore = xSemaphoreCreateBinary();
	BUT4.xSemaphore = xSemaphoreCreateBinary();
	BUT5.xSemaphore = xSemaphoreCreateBinary();
	BUT6.xSemaphore = xSemaphoreCreateBinary();
	BUT7.xSemaphore = xSemaphoreCreateBinary();
	BUT8.xSemaphore = xSemaphoreCreateBinary();
	BUT9.xSemaphore = xSemaphoreCreateBinary();
	BUT10.xSemaphore = xSemaphoreCreateBinary();
	BUT11.xSemaphore = xSemaphoreCreateBinary();
	BUT12.xSemaphore = xSemaphoreCreateBinary();
	
	//io_init();
	

	while(1){
		
		if ( xSemaphoreTake(BUT1.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT1.BUT_NUM, buffer);
			send_command(BUT1.data, buffer);
			send_command(BUT1.data1, buffer);
			send_command(BUT1.data2, buffer);
			bizz(1, 1000,0);
		}
		if ( xSemaphoreTake(BUT2.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT2.BUT_NUM, buffer);
			send_command(BUT2.data, buffer);
			send_command(BUT2.data1, buffer);
			send_command(BUT2.data2, buffer);
			bizz(1, 1200,0);
		}
		if ( xSemaphoreTake(BUT3.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT3.BUT_NUM, buffer);
			send_command(BUT3.data, buffer);
			send_command(BUT3.data1, buffer);
			send_command(BUT3.data2, buffer);
			bizz(1, 1400,0);
		}
		if ( xSemaphoreTake(BUT4.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT4.BUT_NUM, buffer);
			send_command(BUT4.data, buffer);
			send_command(BUT4.data1, buffer);
			send_command(BUT4.data2, buffer);
			bizz(1, 1600,0);
		}
		if ( xSemaphoreTake(BUT5.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT5.BUT_NUM, buffer);
			send_command(BUT5.data, buffer);
			send_command(BUT5.data1, buffer);
			send_command(BUT5.data2, buffer);
			bizz(1, 1700,0);
		}
		if ( xSemaphoreTake(BUT6.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT6.BUT_NUM, buffer);
			send_command(BUT6.data, buffer);
			send_command(BUT6.data1, buffer);
			send_command(BUT6.data2, buffer);
			bizz(1, 1900,0);
		}
		if ( xSemaphoreTake(BUT7.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT7.BUT_NUM, buffer);
			send_command(BUT7.data, buffer);
			send_command(BUT7.data1, buffer);
			send_command(BUT7.data2, buffer);
			bizz(1, 2100,0);
		}
		if ( xSemaphoreTake(BUT8.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT8.BUT_NUM, buffer);
			send_command(BUT8.data, buffer);
			send_command(BUT8.data1, buffer);
			send_command(BUT8.data2, buffer);
			bizz(1, 2400,0);
		}
		if ( xSemaphoreTake(BUT9.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT9.BUT_NUM, buffer);
			send_command(BUT9.data, buffer);
			send_command(BUT9.data1, buffer);
			send_command(BUT9.data2, buffer);
			bizz(1, 2600,0);
		}
		if ( xSemaphoreTake(BUT10.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT10.BUT_NUM, buffer);
			send_command(BUT10.data, buffer);
			send_command(BUT10.data1, buffer);
			send_command(BUT10.data2, buffer);
			bizz(1, 2800,0);
		}
		if ( xSemaphoreTake(BUT11.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT11.BUT_NUM, buffer);
			send_command(BUT11.data, buffer);
			send_command(BUT11.data1, buffer);
			send_command(BUT11.data2, buffer);
			bizz(1, 3400,0);
		}
		if ( xSemaphoreTake(BUT12.xSemaphore, ( TickType_t ) 1) == pdTRUE){
			//send_command(BUT12.BUT_NUM, buffer);
			send_command(BUT12.data, buffer);
			send_command(BUT12.data1, buffer);
			send_command(BUT12.data2, buffer);
			bizz(1, 4000,0);

		}

	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void){
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_PROCESS_STACK_SIZE, NULL,	TASK_PROCESS_STACK_PRIORITY, NULL);
  
	if (xTaskCreate(task_slider, "slider", TASK_SLIDER_STACK_SIZE, NULL, TASK_SLIDER_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test slider task\r\n");
	}
	
	if (xTaskCreate(task_botoes, "botoes", TASK_BOTOES_STACK_SIZE, NULL, TASK_BOTOES_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test boT task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
