/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <stdio.h>
#include <asf.h>
#include <string.h>
#include <assert.h>

#define APERTADO '1'
#define LIBERADO '0'


#define EBUT1_PIO PIOA //start EXT 9 PD28
#define EBUT1_PIO_ID ID_PIOA
#define EBUT1_PIO_IDX 3
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)

#define EBUT2_PIO PIOA //pause  Ext 4 PA19 PA = 10
#define EBUT2_PIO_ID ID_PIOA
#define EBUT2_PIO_IDX 4
#define EBUT2_PIO_IDX_MASK (1u << EBUT2_PIO_IDX)



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

volatile Bool but_status;
volatile Bool g1_is_conversion_done = false;
volatile uint32_t pot_ul_value;
char buffer[6];
char bufferx[6];

typedef void (*p_handler) (uint32_t, uint32_t);

//Define e inicia struct para botoes

typedef struct {
	uint32_t PIO_NAME;
	uint32_t PIO_ID;
	uint32_t PIO_IDX;
	uint32_t PIO_MASK;
	volatile Bool but_flag;
	char BUT_NUM;
} botao;

botao BUT1 = {.PIO_NAME = PIOC, .PIO_ID = ID_PIOC, .PIO_IDX = 17u, .PIO_MASK = (1u << 17u), .BUT_NUM = 'a'};
botao BUT2 = {.PIO_NAME = PIOC, .PIO_ID = ID_PIOC, .PIO_IDX = 30u, .PIO_MASK = (1u << 30u), .BUT_NUM = 'b'};
botao BUT3 = {.PIO_NAME = PIOA, .PIO_ID = ID_PIOA, .PIO_IDX = 3u, .PIO_MASK = (1u << 3u), .BUT_NUM = 'c'};

// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

volatile long g_systimer = 0;

static void AFEC_pot_callback(void)
{
	pot_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT_SENSOR);
	g1_is_conversion_done = true;
}


void but1_callback(void)
{
	BUT1.but_flag = true;
	if(!pio_get(BUT1.PIO_NAME, PIO_INPUT, BUT1.PIO_MASK))
		but_status = APERTADO;
	else 
		but_status = LIBERADO;
}

void but2_callback(void)
{
	BUT2.but_flag = true;
	if(!pio_get(BUT2.PIO_NAME, PIO_INPUT, BUT2.PIO_MASK))
	but_status = APERTADO;
	else
	but_status = LIBERADO;
}

void but3_callback(void)
{
	BUT3.but_flag = true;
	if(!pio_get(BUT3.PIO_NAME, PIO_INPUT, BUT3.PIO_MASK))
	but_status = APERTADO;
	else
	but_status = LIBERADO;
}


void SysTick_Handler() {
	g_systimer++;
}

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	 // RX - PB0  TX - PB1 
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);	
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEHUGO", 1000);
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN4567", 1000);
	usart_log("hc05_server_init", buffer_rx);
}



void send_command(uint32_t nbotao, char buffer[]){
	char eof = 'X';
	buffer[4] = 'x';
	buffer[5] = nbotao;
	while(!usart_is_tx_ready(UART_COMM));
	usart_serial_write_packet(USART0, buffer , 6);
	//usart_write(UART_COMM, nbotao);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, eof);
}

//Funcao que inicializa os botoes e a interrupcao gerada por eles
void iniciabots(botao BOT, p_handler *funcao){		
			
	pio_configure(BOT.PIO_NAME, PIO_INPUT, BOT.PIO_MASK, PIO_PULLUP|PIO_DEBOUNCE);
	pio_set_debounce_filter(BOT.PIO_NAME, BOT.PIO_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BOT.PIO_NAME,
	BOT.PIO_ID,
	BOT.PIO_MASK,
	PIO_IT_FALL_EDGE,
	funcao);

	// Ativa interrupção
	pio_enable_interrupt(BOT.PIO_NAME, BOT.PIO_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BOT.PIO_ID);
	NVIC_SetPriority(BOT.PIO_ID, 4); // Prioridade 4	

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
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_8,	AFEC_pot_callback, 1);

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

}


void io_init(void)
{
	
	iniciabots(BUT1, but1_callback);
	iniciabots(BUT2, but2_callback);
	iniciabots(BUT3, but3_callback);

}


int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	configure_console();
	//printf("teste \n");
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	io_init();
	config_POT();
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT_SENSOR);
	afec_start_software_conversion(AFEC0);
	
	
	while(1) {	
		
		afec_start_software_conversion(AFEC0);
		sprintf(buffer, "%04d",pot_ul_value);
		send_command('n', buffer);
		
		if(BUT1.but_flag) {
			send_command(BUT1.BUT_NUM, buffer);			
			BUT1.but_flag = false;
		}
		if(BUT2.but_flag) {
			send_command(BUT2.BUT_NUM, buffer);
			BUT2.but_flag = false;
		}
		if(BUT3.but_flag) {
			send_command(BUT3.BUT_NUM, buffer);
			BUT3.but_flag = false;
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
		
			

			
		
			
			//pio_clear(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
			//pio_set(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
			//delay_ms(1);
			//pio_clear(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
			//pio_clear(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
			//delay_ms(1);
			
		
		
		//usart_srial_write_packet(USART0, buffer , 5);
		
		//delay_s(0.5);
		//usart_write(UART_COMM, 'X');
		
	}
}
