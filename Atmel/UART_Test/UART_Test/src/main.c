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

#define APERTADO '1'
#define LIBERADO '0'

volatile Bool but_status;

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

botao BUT1 = {.PIO_NAME = PIOC, .PIO_ID = ID_PIOC, .PIO_IDX = 17u, .PIO_MASK = (1u << 17u), .BUT_NUM = '1'};
botao BUT2 = {.PIO_NAME = PIOC, .PIO_ID = ID_PIOC, .PIO_IDX = 30u, .PIO_MASK = (1u << 30u), .BUT_NUM = '2'};
botao BUT3 = {.PIO_NAME = PIOA, .PIO_ID = ID_PIOA, .PIO_IDX = 3u, .PIO_MASK = (1u << 3u), .BUT_NUM = '3'};

// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

volatile long g_systimer = 0;

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



void send_command(int nbotao, int estado){
	char eof = 'X';
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, nbotao);
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
	config_console();
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	io_init();
	
	while(1) {	
		if(BUT1.but_flag) {
			send_command(BUT1.BUT_NUM, but_status);			
			BUT1.but_flag = false;
		}
		if(BUT2.but_flag) {
			send_command(BUT2.BUT_NUM, but_status);
			BUT2.but_flag = false;
		}
		if(BUT3.but_flag) {
			send_command(BUT3.BUT_NUM, but_status);
			BUT3.but_flag = false;
		}
	}
}
