/*MISRA pravila koja nisu ispostovana:
11.2 ignorisemo
2.7 ukazuje na parametar funkcije koji se ne koristi
11.4, 11.6, 14.4, 15.6, 2.2 javljaju se kod davanja i uzimanja semafora
11.1, 4.6, 10.4 kod kreiranja taskova ignorisemo
8.4 u liniji 928 javlja se iz nama nepoznatog razloga
*/

/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <string.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"

/* SERIAL SIMULATOR CHANNEL TO USE */
#define COM_CH_0 (0)
#define COM_CH_1 (1)
#define COM_CH_2 (2)

/* TASK PRIORITIES */
#define	TASK_SERIAL_SEND_PRI	(2 + tskIDLE_PRIORITY )
#define TASK_SERIAL_REC_PRI		(3 + tskIDLE_PRIORITY )
#define	SERVICE_TASK_PRI		(1 + tskIDLE_PRIORITY )

void main_demo(void);

/* TASKS: FORWARD DECLARATIONS */
static void SerialReceiveTask_0(const void* pvParameters); //senzori temperature
static void SerialReceiveTask_1(const void* pvParameters); //zadavanje MINTEMP i MAXTEMP
static void SerialReceiveTask_2(const void* pvParameters); //zadavanje THIGH I TLOW
static void SerialSend_Task0(const void* pvParameters);    //senzori temperature
static void SerialSend_Task2(const void* pvParameters);    //zadavanje THIGH I TLOW
static void prosecna_temp_un(const void* pvParameters);
static void prosecna_temp_sp(const void* pvParameters);
static void kalibracija(const void* pvParameters);
static void mux_seg7_un(const void* pvParameters);   //7-segmentni displej za prikaz unutrasnjih vrednosti
static void mux_seg7_sp(const void* pvParameters);   //7-segmentni displej za prikaz spoljasnjih vrednosti

/* RECEPTION DATA BUFFER */
#define R_BUF_SIZE (32)
static uint16_t prosek_un;
static uint16_t prosek_sp;
static uint16_t sp1;
static uint16_t un1;
static uint16_t promenljiva;
static uint16_t promenljiva2;

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
								0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };

/* GLOBAL OS-HANDLES */
static SemaphoreHandle_t RXC_BS_0, RXC_BS_1, RXC_BS_2;
static SemaphoreHandle_t seg7_un, seg7_sp;
static TimerHandle_t tH1;
static QueueHandle_t otpornost_u;
static QueueHandle_t otpornost_s;
static QueueHandle_t max_otporn;
static QueueHandle_t min_otporn;
static QueueHandle_t min_temperatura;
static QueueHandle_t max_temperatura;
static QueueHandle_t kalibrisana_unutrasnja;
static QueueHandle_t kalibrisana_spoljasnja;
static QueueHandle_t thigh_queue;
static QueueHandle_t tlow_queue;

/* RXC - RECEPTION COMPLETE - INTERRUPT HANDLER */
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_RXC_status(0) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_0, &xHigherPTW) != pdTRUE) {
			printf("Neuspesno slanje podatka - RXC - kanal 0\n");
		}
	}
	if (get_RXC_status(1) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_1, &xHigherPTW) != pdTRUE) {
			printf("Neuspesno slanje podatka - RXC - kanal 1\n");
		}
	}
	if (get_RXC_status(2) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_2, &xHigherPTW) != pdTRUE) {
			printf("Neuspesno slanje podatka - RXC - kanal 2\n");
		}
	}

	portYIELD_FROM_ISR((uint32_t)xHigherPTW);
}

//Taskovi koji omogucavaju automatsko slanje podataka ka PC-u
static void SerialSend_Task0(const void* pvParameters)
{
	uint8_t c = (uint8_t)'u';

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
		if (send_serial_character(COM_CH_0, c) != 0)
		{
			printf("Greska prilikom slanja - kanal 0\n");
		}
	}
}

static void SerialSend_Task2(const void* pvParameters)
{
	uint8_t c = (uint8_t)'t';

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
		if (send_serial_character(COM_CH_2, c) != 0)
		{
			printf("Greska prilikom slanja - kanal 2\n");
		}
	}
}
//funkcija za prijem vrednosti otpornosti sa senzora
static void SerialReceiveTask_0(const void* pvParameters)
{
	static uint16_t r_buffer[R_BUF_SIZE]; //niz za prijem podataka sa kanala1 
	static uint16_t r_point;
	uint8_t cc = 0;
	uint8_t un = 0;
	uint8_t sp = 0;
	uint16_t temperatura_u;
	uint16_t temperatura_s;  
	uint16_t desetica;
	uint16_t jedinica;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BS_0, portMAX_DELAY) != pdTRUE) {
			printf("Neuspesno zauzimanje semafora\n");
		}

		if (get_serial_character(COM_CH_0, &cc) != 0) {
			printf("Neuspesan prijem karaktera sa kanala 0\n");
		}

		//prijem unutrasnje temperature
		if (cc == (uint8_t)'U')
		{
			un = (uint8_t)1;
			un1 = (uint8_t)1;
			sp1 = (uint8_t)0;
			r_point = (uint8_t)0;
		}
		else if (cc == (uint8_t)'S')
		{	
			sp = (uint8_t)1;
			un1 = (uint8_t)0;		
			sp1 = (uint8_t)1;
			r_point = (uint8_t)0;
		}
		else if ((cc == (uint8_t)'F') && (un == (uint8_t)1)) {
			un = (uint8_t)0;
			desetica = (uint16_t)r_buffer[0] - (uint16_t)48; //-48 da bi se dobio decimalni broj
			jedinica = (uint16_t)r_buffer[1] - (uint16_t)48;
			temperatura_u = (desetica * (uint16_t)10) + jedinica;

			if (xQueueSend(otpornost_u, &temperatura_u, 0) != pdTRUE)
			{
				printf("Neuspesno slanje u red otpornost_u\n");
			}
			else {
				printf("Unutrasnja otpornost: %d\n", temperatura_u);
			}

			temperatura_u = (uint16_t)0;

			static uint8_t i = 0;
			for (i = 0; i < r_point; i++) {
				r_buffer[i] = (uint8_t)'\0'; 
			}
		}
		else if ((cc == (uint8_t)'F') && (sp == (uint8_t)1)) {
			sp = (uint8_t)0;
			desetica = (uint16_t)r_buffer[0] - (uint16_t)48; //-48 da bi se dobio decimalni broj
			jedinica = (uint16_t)r_buffer[1] - (uint16_t)48;
			temperatura_s = (desetica * (uint16_t)10) + jedinica;

			if (xQueueSend(otpornost_s, &temperatura_s, 0) != pdTRUE)
			{
				printf("Neuspesno slanje u red otpornost_s\n");
			}
			else {
				printf("Spoljasnja otpornost: %d\n", temperatura_s);
			}

			temperatura_s = (uint16_t)0;

			static uint8_t i = 0;
			for (i = (uint8_t)0; i < r_point; i++) {
				r_buffer[i] = (uint8_t)'\0';
			}
		}
		else {
			r_buffer[r_point] = (uint8_t)cc;
			r_point++;
		}
	}
}
//Task za zadavanje minimalne i maksimalne temperature i otpornosti, potrebnih za kalibraciju
//prijem podataka sa kanala 1, prima se u formatu \00MAXTEMP4012CR - gde je 40 temperatura, a 12 otpornost
static void SerialReceiveTask_1(const void* pvParameters)
{
	static uint16_t r_buffer1[R_BUF_SIZE]; //niz za prijem podataka sa kanala1 
	static uint16_t r_point1;
	uint8_t cc;
	uint16_t max_r;
	uint16_t max_r_desetica;
	uint16_t max_r_jedinica;
	uint16_t min_r;
	uint16_t min_r_desetica;
	uint16_t min_r_jedinica;
	uint16_t min_t;
	uint16_t max_t;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BS_1, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesno davanje semafora za kanal 1\n");
		}

		if (get_serial_character(COM_CH_1, &cc) != 0)
		{
			printf("Neuspesno uzimanje karaktera sa kanala 1\n");
		}

		if (cc == (uint8_t)0x00)
		{
			r_point1 = 0;
		}
		else if (cc == (uint8_t)'CR')
		{
			if ((r_buffer1[0] == (uint16_t)'M') && (r_buffer1[1] == (uint16_t)'I') && (r_buffer1[2] == (uint16_t)'N') && (r_buffer1[3] == (uint16_t)'T') && (r_buffer1[4] == (uint16_t)'E') && (r_buffer1[5] == (uint16_t)'M') && (r_buffer1[6] == (uint16_t)'P')) {
				//prijem temperature
				max_r_desetica = (uint16_t)r_buffer1[7] - (uint16_t)48; //-48 da bi se dobio decimalni broj
				max_r_jedinica = (uint16_t)r_buffer1[8] - (uint16_t)48;
				min_t = (max_r_desetica * (uint16_t)10) + max_r_jedinica;

				if (xQueueSend(min_temperatura, &min_t, 0) != pdTRUE)
				{
					printf("Neuspesno slanje u red min_temperatura");
				}
				min_t = (uint16_t)0;

				//prijem otpornosti
				max_r_desetica = (uint16_t)r_buffer1[9] - (uint16_t)48; //-48 da bi se dobio dec broj
				max_r_jedinica = (uint16_t)r_buffer1[10] - (uint16_t)48;
				max_r = (max_r_desetica * (uint16_t)10) + max_r_jedinica;
				if (xQueueSend(max_otporn, &max_r, 0) != pdTRUE)
				{
					printf("Neuspesno slanje u red max_otpornost");
				}
				max_r = (uint16_t)0;
			}
			else if ((r_buffer1[0] == (uint16_t)'M') && (r_buffer1[1] == (uint16_t)'A') && (r_buffer1[2] == (uint16_t)'X') && (r_buffer1[3] == (uint16_t)'T') && (r_buffer1[4] == (uint16_t)'E') && (r_buffer1[5] == (uint16_t)'M') && (r_buffer1[6] == (uint16_t)'P')) {
				min_r_desetica = (uint16_t)r_buffer1[7] - (uint16_t)48; //-48 da bi se dobio dec broj
				min_r_jedinica = (uint16_t)r_buffer1[8] - (uint16_t)48;
				max_t = (min_r_desetica * (uint16_t)10) + min_r_jedinica;

				if (xQueueSend(max_temperatura, &max_t, 0) != pdTRUE)
				{
					printf("Neuspesno slanje u red max_temperatura");
				}
				max_t = (uint16_t)0;

				min_r_desetica = (uint16_t)r_buffer1[9] - (uint16_t)48; //-48 da bi se dobio dec broj
				min_r_jedinica = (uint16_t)r_buffer1[10] - (uint16_t)48;
				min_r = (min_r_desetica * (uint16_t)10) + min_r_jedinica;

				if (xQueueSend(min_otporn, &min_r, 0) != pdTRUE)
				{
					printf("Neuspesno slanje u red min_otpornost");
				}

				min_r = (uint16_t)0;
			}
			else {
				printf("Pogresan format pri upisu na kanal 1\n");
			}

			static uint8_t m = 0;
			for (m = (uint8_t)0; m < (uint8_t)10; m++) {
				r_buffer1[m] = (uint16_t)'\0';
			}
		}
		else
		{
			r_buffer1[r_point1] = (uint8_t)cc;
			r_point1++;
		}
	}
}

//Task za ogranicavanje opsega temperature
//Prijem podataka sa kanala 2, prima se u formatu T3505F, gde je THIGH = 35, a TLOW = 05
static void SerialReceiveTask_2(const void* pvParameters)
{
	static uint16_t r_buffer2[R_BUF_SIZE]; //niz za prijem podataka sa kanala1 
	static uint16_t r_point2;
	uint8_t cc = 0;
	uint16_t thigh;
	uint16_t thigh_desetica;
	uint16_t thigh_jedinica;
	uint16_t tlow;
	uint16_t tlow_desetica;
	uint16_t tlow_jedinica;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BS_2, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesno davanje semafora za kanal 2\n");
		}

		if (get_serial_character(COM_CH_2, &cc) != 0)
		{
			printf("Neuspesno uzimanje karaktera sa kanala 2\n");
		}

		if (cc == (uint8_t)'T') {
			r_point2 = (uint16_t)0;
		}
		else if ((cc == (uint8_t)'F')) {

			thigh_desetica = (uint16_t)r_buffer2[0] - (uint16_t)48; //-48 da bi se dobio decimalni broj
			thigh_jedinica = (uint16_t)r_buffer2[1] - (uint16_t)48;
			thigh = (thigh_desetica * (uint16_t)10) + thigh_jedinica;

			tlow_desetica = (uint16_t)r_buffer2[2] - (uint16_t)48; //-48 da bi se dobio decimalni broj
			tlow_jedinica = (uint16_t)r_buffer2[3] - (uint16_t)48;
			tlow = (tlow_desetica * (uint16_t)10) + tlow_jedinica;

			if (xQueueSend(thigh_queue, &thigh, 0) != pdTRUE)
			{
				//printf("Neuspesno slanje u red thigh_queue\n");
			}
			else {
				printf("THIGH: %d\n", thigh);
			}

			if (xQueueSend(tlow_queue, &tlow, 0) != pdTRUE)
			{
				//printf("Neuspesno slanje u red tlow_queue\n");
			}
			else {
				printf("TLOW: %d\n", tlow);
			}

			static uint8_t i = 0;
			for (i = 0; i < r_point2; i++) {
				r_buffer2[i] = (uint8_t)'\0';
			}
		}
		else {
			r_buffer2[r_point2] = (uint8_t)cc;
			r_point2++;
		}
	}
}

//Task za izracunavanje prosecne unutrasnje temperature na osnovu 5 uzastopnih merenja
static void prosecna_temp_un(const void* pvParameters)
{
	uint16_t t_un;
	uint16_t brojac = 0;
	uint16_t zbir = 0;

	for (;;)
	{
		if (xQueueReceive(otpornost_u, &t_un, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda otpornost_u");
		}

		if ((t_un > (uint16_t)0) && (t_un < (uint16_t)33)) {
			zbir = zbir + t_un;
			brojac++;
			if (brojac == (uint16_t)5)
			{
				prosek_un = zbir / (uint16_t)5;
				brojac = (uint16_t)0;
				zbir = (uint16_t)0;
			}
		}
		else {
			printf("Unutrasnja otpornost nije u zeljenom opsegu. \n");
		}
	}
}

//Task za izracunavanje prosecne spoljasnje temperature na osnovu 5 uzastopnih merenja
static void prosecna_temp_sp(const void* pvParameters)
{
	uint16_t t_sp;
	uint16_t brojac = 0;
	uint16_t zbir = 0;

	for (;;)
	{
		if (xQueueReceive(otpornost_s, &t_sp, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda otpornost_s");
		}

		if ((t_sp > (uint16_t)0) && (t_sp < (uint16_t)33)) {
			zbir = zbir + t_sp;
			brojac++;
			if (brojac == (uint16_t)5)
			{
				prosek_sp = zbir / (uint16_t)5;
				brojac = 0;
				zbir = 0;
			}
		}
		else {
			printf("Spoljasnja otpornost nije u zeljenom opsegu. \n");
		}
	}
}

//Task u kojem se vrsi kalibracija temperature na osnovu vrednosti otpornosti unesene preko kanala 0
//U ovom tasku realizovana je i signalizacija prekoracenja opsega dozvoljene temperature preko LED bara
static void kalibracija(const void* pvParameters) {
	
	printf("KALIBRACIJA\n");

	static uint16_t max_otp;
	static uint16_t min_otp;
	static uint16_t max_tem;
	static uint16_t min_tem;
	static uint16_t THIGH;
	static uint16_t TLOW;
	static uint16_t trenutna_temp_un;
	static uint16_t trenutna_temp_sp;
	uint8_t  brojac;

	for (;;) {
		if (xQueueReceive(thigh_queue, &THIGH, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda thigh_queue\n");
		}
		if (xQueueReceive(tlow_queue, &TLOW, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda tlow_queue\n");
		}
		if (xQueueReceive(max_otporn, &max_otp, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost");
		}
		if (xQueueReceive(min_otporn, &min_otp, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda min_otpornost");
		}
		if (xQueueReceive(max_temperatura, &max_tem, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_temperatura");
		}
		if (xQueueReceive(min_temperatura, &min_tem, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda min_temperatura");
		}
		printf("Min otpornost iz reda: %d\n", min_otp);
		printf("Max otpornost iz reda: %d\n", max_otp);
		printf("Min temperatura iz reda: %d\n", min_tem);
		printf("Max temperatura iz reda: %d\n", max_tem);

		if ((prosek_un != (uint16_t)0) && (un1 == (uint16_t)1)) {
			trenutna_temp_un = ((min_tem - max_tem) * (prosek_un - min_otp) / (max_otp - min_otp)) + max_tem;
			
			if (xQueueSend(kalibrisana_unutrasnja, &trenutna_temp_un, 0) != pdTRUE)
			{
				printf("Neuspesno slanje u red kalibrisana_unutrasnja\n");
			}

			prosek_un = 0;
			prosek_sp = 0;
			un1 = 0;
		}
		
		if ((prosek_sp != (uint16_t)0) && (sp1 == (uint16_t)1)) {

			trenutna_temp_sp = ((min_tem - max_tem) * (prosek_sp - min_otp) / (max_otp - min_otp)) + max_tem;
			
			if (xQueueSend(kalibrisana_spoljasnja, &trenutna_temp_sp, 0) != pdTRUE)
			{
				printf("Neuspesno slanje u red kalibrisana_spoljasnja\n");
			}
		
			prosek_sp = 0;
			prosek_un = 0;
			sp1 = 0;
		}

		printf("Kalibrisana temp unutrasnja: %d\n", trenutna_temp_un);
		printf("Kalibrisana temp spoljasnja: %d\n", trenutna_temp_sp);

		//Signalizacija na led baru ako temperatura nije u zeljenom opsegu
		if (((trenutna_temp_un < (uint16_t)TLOW) || (trenutna_temp_un > (uint16_t)THIGH)) && ((trenutna_temp_sp < (uint16_t)TLOW) || (trenutna_temp_sp > (uint16_t)THIGH))) {
			for (brojac = (uint8_t)0; brojac < (uint8_t)3; brojac++) { //ako temperatura nije u zeljenom opsegu, 
																	   //led bar blinka 3 puta
				if (set_LED_BAR(0, 0xff) != 0) {
					printf("Nije ukljucio led bar\n");
				}
				if (set_LED_BAR(1, 0xff) != 0) {
					printf("Nije ukljucio led bar\n");
				}
				vTaskDelay(pdMS_TO_TICKS(500));

				if (set_LED_BAR(0, 0x00) != 0) {
					printf("Nije ukljucio led bar\n");
				}
				if (set_LED_BAR(1, 0x00) != 0) {
					printf("Nije ukljucio led bar\n");
				}
				vTaskDelay(pdMS_TO_TICKS(500));
			}
		}
		else if ((trenutna_temp_un < (uint16_t)TLOW) || (trenutna_temp_un > (uint16_t)THIGH)) {

			for (brojac = (uint8_t)0; brojac < (uint8_t)3; brojac++) {
		
				if (set_LED_BAR(0, 0xff) != 0) {
					printf("Nije ukljucio led bar\n");
				}

				vTaskDelay(pdMS_TO_TICKS(500));

				if (set_LED_BAR(0, 0x00) != 0) {
					printf("Nije ukljucio led bar\n");
				}

				vTaskDelay(pdMS_TO_TICKS(500));
			}
		}
		else if ((trenutna_temp_sp < (uint16_t)TLOW) || (trenutna_temp_sp > (uint16_t)THIGH)) {

			for (brojac = (uint8_t)0; brojac < (uint8_t)3; brojac++) {
				// printf("LED sija 1\n");
				if (set_LED_BAR(1, 0xff) != 0) {
					printf("Nije ukljucio led bar\n");
				}

				vTaskDelay(pdMS_TO_TICKS(500));

				if (set_LED_BAR(1, 0x00) != 0) {
					printf("Nije ukljucio led bar\n");
				}

				vTaskDelay(pdMS_TO_TICKS(500));
			}
		}
		else {
			printf("Obe temperature su u zeljenom opsegu\n");
		}
	}
}

//Funkcija koja se poziva svakih 100ms i proverava stanje tastera na led baru
static void timer_seg7(const TimerHandle_t*tm) {

	static uint8_t dd;
	
	if (get_LED_BAR(2, &dd) != 0)  //uzimamo podatke o pritisnutim tasterima sa treceg stupca
	{
		//printf("Greska prilikom ocitavanja led bara\n");
	}
	
	if ((dd == (uint8_t)1) && (promenljiva == (uint8_t)0)) {
		promenljiva = 1; //signalizira da je samo jednom pritisnut taster
		dd = 0;
		if ( xSemaphoreGive(seg7_un, portMAX_DELAY) != pdTRUE ) {
			printf("Neuspesno davanje semafora seg7_un\n");
		}
	}
	else if ((dd == (uint8_t)2) && (promenljiva2 == (uint8_t)0)) {
		promenljiva2 = 1;  //signalizira da je samo jednom pritisnut taster
		dd = 0;
		if (xSemaphoreGive(seg7_sp, portMAX_DELAY) != pdTRUE) {
			printf("Neuspesno davanje semafora seg7_sp\n");
		}
	}
	//ovaj else je dodat samo zbog MISRA pravila, a poruka je zakomentarisana da ne opterecuje terminal
	else { 
		//printf("nije stisnut nijedan taster\n");
	}
}

//Task pomocu kojeg se ispisuju vrednosti unutrasnje temperature i otpornosti na seg7_mux
static void mux_seg7_un(const void* pvParameters) {
	
	uint16_t ku;
	uint16_t ou;
	uint16_t desetica_un_t;
	uint16_t jedinica_un_t;
	uint16_t desetica_un_o;
	uint16_t jedinica_un_o;
	
	for (;;) {
	
		if (xQueueReceive(kalibrisana_unutrasnja, &ku, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost\n");
		}

		if (xQueueReceive(otpornost_u, &ou, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost\n");
		}

		desetica_un_t = ku / (uint16_t)10;		
		jedinica_un_t = ku % (uint16_t)10;		
		desetica_un_o = ou / (uint16_t)10;		
		jedinica_un_o = ou % (uint16_t)10;
		
		promenljiva = 0;

		if (xSemaphoreTake(seg7_un, portMAX_DELAY) != pdTRUE) {
			printf("Neuspesno TAKE semafora seg7_un\n");
		} 
		else {

			if (select_7seg_digit(0) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(0x3e) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
			if (select_7seg_digit(2) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(hexnum[desetica_un_t]) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
			if (select_7seg_digit(3) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(hexnum[jedinica_un_t]) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
			//ispis otpornosti
			if (select_7seg_digit(5) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(hexnum[desetica_un_o]) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
			if (select_7seg_digit(6) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(hexnum[jedinica_un_o]) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
		}
	}
}

//Task pomocu kojeg se ispisuju vrednosti spoljasnje temperature i otpornosti na seg7_mux
static void mux_seg7_sp(const void* pvParameters) {
	uint16_t desetica_sp_t;
	uint16_t jedinica_sp_t;
	uint16_t desetica_sp_o;
	uint16_t jedinica_sp_o;
	uint16_t os;
	uint16_t ks;

	for (;;) {
	
		if (xQueueReceive(kalibrisana_spoljasnja, &ks, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost\n");
		}
		
		if (xQueueReceive(otpornost_s, &os, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost\n");
		}

		desetica_sp_t = ks / (uint16_t)10;		
		jedinica_sp_t = ks % (uint16_t)10;
		desetica_sp_o = os / (uint16_t)10;
		jedinica_sp_o = os % (uint16_t)10;
		
		promenljiva2 = 0;

		if (xSemaphoreTake(seg7_sp, portMAX_DELAY) != pdTRUE) {
			printf("Neuspesno TAKE semafora seg7_sp\n");
		}
		else {
			//ispis temperature
			if (select_7seg_digit(0) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(0x6d) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
			if (select_7seg_digit(2) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(hexnum[desetica_sp_t]) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
			if (select_7seg_digit(3) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(hexnum[jedinica_sp_t]) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
			//ispis otpornosti
			if (select_7seg_digit(5) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(hexnum[desetica_sp_o]) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
			if (select_7seg_digit(6) != 0)
			{
				printf("Greska prilikom selektovanja mux_7seg\n");
			}
			if (set_7seg_digit(hexnum[jedinica_sp_o]) != 0)
			{
				printf("Greska prilikom setovanja mux_7seg\n");
			}
		}
	}
}



/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void)
{
	BaseType_t status;

	//Inicijalizacija LED bara
	if (init_LED_comm() != 0) {
		printf("Neuspesna inicijalizacija \n");
	}
	// inicijalizacija serijske TX na kanalu 0
	if (init_serial_uplink(COM_CH_0) != 0) {
		printf("Neuspesna inicijalizacija \n");
	}
	// inicijalizacija serijske RX na kanalu 0
	if (init_serial_downlink(COM_CH_0) != 0) {
		printf("Neuspesna inicijalizacija \n");
	}
	// inicijalizacija serijske TX na kanalu 1
	if (init_serial_uplink(COM_CH_1) != 0) {
		printf("Neuspesna inicijalizacija \n");
	}
	// inicijalizacija serijske RX na kanalu 1
	if (init_serial_downlink(COM_CH_1) != 0) {
		printf("Neuspesna inicijalizacija \n");
	}
	// inicijalizacija serijske TX na kanalu 2
	if (init_serial_uplink(COM_CH_2) != 0) {
		printf("Neuspesna inicijalizacija \n");
	}
	// inicijalizacija serijske RX na kanalu 2
	if (init_serial_downlink(COM_CH_2) != 0) {
		printf("Neuspesna inicijalizacija \n");
	}
	// inicijalizacija seg7_mux
	if (init_7seg_comm() != 0) {
		printf("Neuspesna inicijalizacija \n");
	}

	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	/* Create TBE semaphores - serial transmit comm */
	RXC_BS_0 = xSemaphoreCreateBinary();
	RXC_BS_1 = xSemaphoreCreateBinary();
	RXC_BS_2 = xSemaphoreCreateBinary();
	seg7_un = xSemaphoreCreateBinary();
	seg7_sp = xSemaphoreCreateBinary();

	/* SERIAL RECEIVER TASK */
	status = xTaskCreate(SerialReceiveTask_0, "SR0", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != (BaseType_t)pdPASS)
	{
		printf("Greska prilikom kreiranja SerialReceive_Task0\n");
	}

	/* SERIAL RECEIVER TASK */
	
	status = xTaskCreate(SerialReceiveTask_1, "SR1", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja Serialreceive_Task1\n");
	}
	
	status = xTaskCreate(SerialReceiveTask_2, "SR2", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI+1, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja Serialreceive_Task2\n");
	}

	/* SERIAL TRANSMITTER TASK */
	status = xTaskCreate(SerialSend_Task0, "ST0", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja SerialSend_Task0\n");
	}
	status = xTaskCreate(SerialSend_Task2, "STx2", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja SerialSend_Task2\n");
	}

	//SERVICE TASKS
	status = xTaskCreate(prosecna_temp_un, "prosecna_temp_un", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska prosecna_temp_un\n");
	}
	status = xTaskCreate(prosecna_temp_sp, "prosecna_temp_sp", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska prosecna_temp_sp\n");
	}
	status = xTaskCreate(kalibracija, "kalibracija", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska kalibracija_sp\n");
	}
	status = xTaskCreate(mux_seg7_un, "mux_seg7_un", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska mux_seg7_un\n");
	}
	status = xTaskCreate(mux_seg7_sp, "mux_seg7_sp", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska mux_seg7_sp\n");
	}

	//QUEUES
	otpornost_u = xQueueCreate(10, sizeof(uint16_t)); 
	if (otpornost_u == NULL)
	{
		printf("Greska prilikom kreiranja reda otpornost_u\n");
	}
	otpornost_s = xQueueCreate(10, sizeof(uint16_t)); 
	if (otpornost_s == NULL)
	{
		printf("Greska prilikom kreiranja reda otpornost_s\n");
	}
	max_otporn = xQueueCreate(10, sizeof(uint16_t)); 
	if (max_otporn == NULL)
	{
		printf("Greska prilikom kreiranja reda max_otpornost\n");
	}
	min_otporn = xQueueCreate(10, sizeof(uint16_t)); 
	if (min_otporn == NULL)
	{
		printf("Greska prilikom kreiranja reda min_otpornost\n");
	}
	max_temperatura = xQueueCreate(10, sizeof(uint16_t)); 
	if (max_temperatura == NULL)
	{
		printf("Greska prilikom kreiranja reda max_temperatura\n");
	}
	min_temperatura = xQueueCreate(10, sizeof(uint16_t)); 
	if (min_temperatura == NULL)
	{
		printf("Greska prilikom kreiranja reda min_temperatura\n");
	}
	kalibrisana_unutrasnja = xQueueCreate(10, sizeof(uint16_t)); 
	if (kalibrisana_unutrasnja == NULL)
	{
		printf("Greska prilikom kreiranja reda kalibrisana_unutrasnja\n");
	}
	kalibrisana_spoljasnja = xQueueCreate(10, sizeof(uint16_t));
	if (kalibrisana_spoljasnja == NULL)
	{
		printf("Greska prilikom kreiranja reda kalibrisana_spoljasnja\n");
	}
	thigh_queue = xQueueCreate(10, sizeof(uint16_t));
	if (thigh_queue == NULL)
	{
		printf("Greska prilikom kreiranja reda thigh_queue\n");
	}
	tlow_queue = xQueueCreate(10, sizeof(uint16_t));
	if (tlow_queue == NULL)
	{
		printf("Greska prilikom kreiranja reda tlow_queue\n");
	}

	// Timers
	tH1 = xTimerCreate(
		"test timer",
		pdMS_TO_TICKS(100),
		pdTRUE,
		NULL,
		timer_seg7
	);

	if (xTimerStart(tH1, 0)  != pdPASS) {
		printf("Greska prilikom pokretanja tajmera\n");
	}

	vTaskStartScheduler();

	for (;;) {}
}
