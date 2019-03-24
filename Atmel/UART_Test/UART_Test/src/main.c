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
#include <asf.h>
#include <string.h>

// defines do Botao de start
#define BUTTSTART_PIO           PIOC
#define BUTTSTART_PIO_ID        ID_PIOC
#define BUTTSTART_PIO_IDX       17u
#define BUTTSTART_PIO_IDX_MASK  (1u << BUTTSTART_PIO_IDX)

#define BUTTSTART1_PIO           PIOC
#define BUTTSTART1_PIO_ID        ID_PIOC
#define BUTTSTART1_PIO_IDX       30u
#define BUTTSTART1_PIO_IDX_MASK  (1u << BUTTSTART1_PIO_IDX)

#define BUTTSTART2_PIO           PIOA
#define BUTTSTART2_PIO_ID        ID_PIOA
#define BUTTSTART2_PIO_IDX       3u
#define BUTTSTART2_PIO_IDX_MASK  (1u << BUTTSTART2_PIO_IDX)

#define APERTADO '1'
#define LIBERADO '0'

#define BUT_ID '1'
#define BUT1_ID '2'
#define BUT2_ID '3'
#define BUT3_ID '4'
#define BUT4_ID '5'
#define BUT5_ID '6'

// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL

volatile Bool but_status;
volatile Bool but_flag;
volatile Bool but1_flag;
volatile Bool but2_flag;
volatile Bool but3_flag;
volatile Bool but4_flag;


#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

volatile long g_systimer = 0;

void but_callback(void)
{
	but_flag = true;
	if(!pio_get(BUTTSTART_PIO, PIO_INPUT, BUTTSTART_PIO_IDX_MASK))
		but_status = APERTADO;
	else 
		but_status = LIBERADO;
}

void but1_callback(void)
{
	but1_flag = true;
	if(!pio_get(BUTTSTART1_PIO, PIO_INPUT, BUTTSTART1_PIO_IDX_MASK))
	but_status = APERTADO;
	else
	but_status = LIBERADO;
}

void but2_callback(void)
{
	but2_flag = true;
	if(!pio_get(BUTTSTART2_PIO, PIO_INPUT, BUTTSTART2_PIO_IDX_MASK))
	but_status = APERTADO;
	else
	but_status = LIBERADO;
}


void SysTick_Handler() {
	g_systimer++;
}

void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
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



void send_command(int botao, int estado){
	char eof = 'X';
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, botao);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, eof);
	delay_ms(200);
}


void io_init(void)
{

	// Configura led
	//pmc_enable_periph_clk(LED_PIO_ID);
	//pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUTTSTART_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUTTSTART_PIO, PIO_INPUT, BUTTSTART_PIO_IDX_MASK, PIO_PULLUP);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUTTSTART_PIO,
	BUTTSTART_PIO_ID,
	BUTTSTART_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_callback);

	// Ativa interrupção
	pio_enable_interrupt(BUTTSTART_PIO, BUTTSTART_PIO_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUTTSTART_PIO_ID);
	NVIC_SetPriority(BUTTSTART_PIO_ID, 4); // Prioridade 4
	//---------------------------------------------------------------------------------------------------------------
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUTTSTART1_PIO,
	BUTTSTART1_PIO_ID,
	BUTTSTART1_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but1_callback);

	// Ativa interrupção
	pio_enable_interrupt(BUTTSTART1_PIO, BUTTSTART1_PIO_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUTTSTART1_PIO_ID);
	NVIC_SetPriority(BUTTSTART1_PIO_ID, 4); // Prioridade 4
	//----------------------------------------------------------------------------------------------------------------
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUTTSTART2_PIO,
		BUTTSTART2_PIO_ID,
		BUTTSTART2_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but2_callback);

	// Ativa interrupção
	pio_enable_interrupt(BUTTSTART2_PIO, BUTTSTART2_PIO_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUTTSTART2_PIO_ID);
	NVIC_SetPriority(BUTTSTART2_PIO_ID, 4); // Prioridade 4


}




int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	io_init();
	
	while(1) {	
		if(but_flag) {
			send_command(BUT_ID, but_status);			
			but_flag = false;
		}
		if(but1_flag) {
			send_command(BUT1_ID, but_status);
			but1_flag = false;
		}
		if(but2_flag) {
			send_command(BUT2_ID, but_status);
			but2_flag = false;
		}
	}
}
