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
#define TLOW 5
#define THIGH 35
// #define COM_CH_2 (2)

/* TASK PRIORITIES */
#define	TASK_SERIAL_SEND_PRI	(2 + tskIDLE_PRIORITY )
#define TASK_SERIAL_REC_PRI		(3 + tskIDLE_PRIORITY )
#define	SERVICE_TASK_PRI		(1 + tskIDLE_PRIORITY )
void main_demo(void);
/* TASKS: FORWARD DECLARATIONS */
static void SerialReceiveTask_0(void* pvParameters); //senzor temperature unutrasnji
static void SerialReceiveTask_1(void* pvParameters); //senzor temperature spoljasnji
static void SerialSend_Task0(void* pvParameters);    //senzor temperature unutrasnji
//static void SerialSend_Task1(void* pvParameters);    //senzor temperature spoljasnji
static void prosecna_temp_un(void* pvParameters);
static void prosecna_temp_sp(void* pvParameters);
static void kalibracija(void* pvParameters);
//static void led_bar_tsk1(const void* pvParameters);   //ulazi-izlazi
//static void led_bar_tsk2(const void* pvParameters);
static void mux_seg7(const void* pvParameters);       //7-segmentni displej
static void mux_seg7_sp(const void* pvParameters);       //7-segmentni displej za prikaz spoljasnjih vrednosti

/* RECEPTION DATA BUFFER */
#define R_BUF_SIZE (32)
static uint8_t r_buffer[R_BUF_SIZE]; //niz za prijem podataka sa kanala0 (senzor temperature unutrasnji)
static uint16_t r_buffer1[R_BUF_SIZE]; //niz za prijem podataka sa kanala1 (senzor temperature spoljasnji)
static uint8_t volatile r_point;     //brojac za kanal0
static uint16_t volatile r_point1;     //brojac za kanal1
static uint16_t temperatura_u; //unutrasnja temperatura
static uint16_t temperatura_s;  //spoljasnja temperatura
//static uint16_t podatak;  //podatak sa serijske
static uint16_t max_r;
static uint16_t max_r_desetica; 
static uint16_t max_r_jedinica; 
static uint16_t min_r;  
static uint16_t min_r_desetica;
static uint16_t min_r_jedinica;
static uint16_t prosek_un;
static uint16_t prosek_sp;
static uint16_t trenutna_temp_un;
static uint16_t trenutna_temp_sp;
static uint16_t min_t;
static uint16_t max_t;
static uint16_t sp1;
static uint16_t un1;
static uint16_t promenljiva;
static uint16_t promenljiva2;

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
								0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };

/* GLOBAL OS-HANDLES */
//static SemaphoreHandle_t LED_INT_BinarySemaphore;
//static SemaphoreHandle_t LED_INT_BinarySemaphore2;
static SemaphoreHandle_t RXC_BS_0, RXC_BS_1;
static SemaphoreHandle_t seg7_un, seg7_sp;
//static SemaphoreHandle_t RXC_BS_2;
//static SemaphoreHandle_t TBE_BinarySemaphore;
static TimerHandle_t tH1;
static TimerHandle_t tH2;
static QueueHandle_t otpornost_u;
static QueueHandle_t otpornost_s;
static QueueHandle_t max_otporn;
static QueueHandle_t min_otporn;
static QueueHandle_t min_temperatura;
static QueueHandle_t max_temperatura;
static QueueHandle_t kalibrisana_unutrasnja;
static QueueHandle_t kalibrisana_spoljasnja;
static QueueHandle_t seg7;
/*
static uint32_t OnLED_ChangeInterrupt() {

	static BaseType_t higherPriorityTaskWoken;
	higherPriorityTaskWoken = pdFALSE;

	if (xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &higherPriorityTaskWoken) != pdTRUE) {
		printf("Neuspesno slanju podatka - LED\n");
	}
		portYIELD_FROM_ISR((uint32_t)higherPriorityTaskWoken);
}
*/
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
	/*if (get_RXC_status(2) != 0) {
		if (xSemaphoreGiveFromISR(RXC_BS_2, &xHigherPTW) != pdTRUE) {
			printf("Neuspesno slanje podatka - RXC - kanal 2\n");
		}
	}*/

	portYIELD_FROM_ISR((uint32_t)xHigherPTW);
}



//funkcija za prijem vrednosti temperature sa senzora unutrasnjeg
static void SerialReceiveTask_0(void* pvParameters)
{
	uint8_t cc = 0;
	static uint8_t un = 0;
	static uint8_t sp = 0;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BS_0, portMAX_DELAY) != pdTRUE) {
			printf("Neuspesno zauzimanje semafora\n");
		}

		if (get_serial_character(COM_CH_0, &cc) != 0) {
			//	printf("Neuspesan prijem karaktera sa kanala 0\n");
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
			un1 = (uint8_t)0;
			sp = (uint8_t)1;
			sp1 = (uint8_t)1;
			r_point = (uint8_t)0;
		}
		else if ((cc == (uint8_t)'F') && (un == (uint8_t)1)) {
			un = (uint8_t)0;
			char* ostatak;
			temperatura_u = (uint32_t)strtol(r_buffer, &ostatak, 10);
			//printf("Unutrasnja temperatura je: %d\n", temperatura_u);

			if (xQueueSend(otpornost_u, &temperatura_u, 0) != pdTRUE)
			{
				printf("Neuspesno slanje u red otpornost_u\n");
			}
			else printf("poslao otpornost unutrasnju %d\n", temperatura_u);
			temperatura_u = (uint16_t)0;
			static uint8_t i = 0;
			for (i = 0; i < r_point; i++) {
				r_buffer[i] = (uint8_t)'\0'; //DODATO (uint8_t) ZBOG MISRE
			}
		}
		else if ((cc == (uint8_t)'F' && sp == (uint8_t)1)) {
			sp = (uint8_t)0;
			char* ostatak;
			temperatura_s = (uint16_t)strtol(r_buffer, &ostatak, 10);
			//printf("Spoljasnja temperatura je: %d\n", temperatura_s);

			if (xQueueSend(otpornost_s, &temperatura_s, 0) != pdTRUE)
			{
				printf("Neuspesno slanje u red otpornost_s\n");
			}
			else printf("poslao otpornost spoljasnju %d\n", temperatura_s);

			temperatura_s = (uint16_t)0;
			static uint8_t i = 0;
			for (i = (uint8_t)0; i < r_point; i++) {
				r_buffer[i] = (uint8_t)'\0';//DODATO (uint8_t) ZBOG MISRE
			}
		}
		else {
			r_buffer[r_point] = (uint8_t)cc; //OVO ZBOG MISRE RAZDVOJENO
			r_point++;
		}
	}
}
/*
void SerialReceiveTask_2(void* pvParameters)
{
	uint8_t cc = 0;

	while (1)
	{
		xSemaphoreTake(RXC_BS_2, portMAX_DELAY);
		//printf("primio karakter: %u\n", (unsigned)cc);
	}
}
*/

//prijem podataka sa kanala 1, prima se u formatu \00MAXTEMP4012CR - gde je 40 temperatura, a 12 otpornost
static void SerialReceiveTask_1(void* pvParameters)
{
	uint8_t cc;

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

			//U UPUTSTVU OBAVEZNO STAVITI DA SE KUCA DVE CIFRE ZA BROJ, AKO PISEM 5 = 05
			if ((r_buffer1[0] == (uint16_t)'M') && (r_buffer1[1] == (uint16_t)'I') && (r_buffer1[2] == (uint16_t)'N') && (r_buffer1[3] == (uint16_t)'T') && (r_buffer1[4] == (uint16_t)'E') && (r_buffer1[5] == (uint16_t)'M') && (r_buffer1[6] == (uint16_t)'P')) {
				//prijem temperature
				max_r_desetica = (uint16_t)r_buffer1[7] - (uint16_t)48; //-48 da bi se dobio dec broj
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

static void SerialSend_Task0(void* pvParameters)
{
	uint8_t c = (uint8_t)'u';

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
		if (send_serial_character(COM_CH_0, c) != 0)
		{
			printf("Greska prilikom slanja");
		}
	}
}
/*
static void SerialSend_Task1(void* pvParameters)
{
}*/
//Task za izracunavanje prosecne unutrasnje temperature na osnovu 5 uzastopnih merenja
static void prosecna_temp_un(void* pvParameters)
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
				//printf("Prosek unutrasnje %d\n", prosek_un);
				brojac = (uint16_t)0;
				zbir = (uint16_t)0;
				/*if (xQueueSend(prosek_unutrasnji, &prosek_un, 0) != pdTRUE)
				{
					printf("Neuspesno slanje u red prosek_unutrasnji\n");
				}*/
			}
		}
		else {
			printf("Unutrasnja otpornost nije u zeljenom opsegu. \n");
		}
	}
}

//Task za izracunavanje prosecne spoljasnje temperature na osnovu 5 uzastopnih merenja
static void prosecna_temp_sp(void* pvParameters)
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
				//printf("Prosek spoljasnje %d\n", prosek_sp);
				brojac = 0;
				zbir = 0;

				/*if (xQueueSend(prosek_spoljasnji, &prosek_sp, 0) != pdTRUE)
				{
					printf("Neuspesno slanje u red prosek_spoljasjni\n");
				}*/
			}
		}
		else {
			printf("Spoljasnja otpornost nije u zeljenom opsegu. \n");
		}
	}
}


static void kalibracija(void* pvParameters) {
	
	printf("SPOLJASNJA KALIBRACIJA\n");
	static uint16_t max_otp;
	static uint16_t min_otp;
	static uint16_t max_tem;
	static uint16_t min_tem;
	uint8_t  brojac;
	//uint16_t p_sp;
	for (;;) {
		/*if (xQueueReceive(prosek_spoljasnji, &p_sp, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda prosek_unutrasnji\n");
		}*/
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
			
			//min_tem = 0;
			//max_tem = 0;
			if (xQueueSend(kalibrisana_unutrasnja, &trenutna_temp_un, 0) != pdTRUE)
			{
				printf("Neuspesno slanje u red kalibrisana_unutrasnja\n");
			}
			else {
				printf("Poslao u red kalibrisana_unutrasnja\n");
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
			else {
				printf("Poslao u red kalibrisana_spoljasnja\n");
			}

			prosek_sp = 0;
			prosek_un = 0;
			sp1 = 0;
		}


		printf("Kalibrisana temp unutrasnja: %d\n", trenutna_temp_un);
		printf("Kalibrisana temp spoljasnja: %d\n", trenutna_temp_sp);

		if (((trenutna_temp_un < (uint16_t)TLOW) || (trenutna_temp_un > (uint16_t)THIGH)) && ((trenutna_temp_sp < (uint16_t)TLOW) || (trenutna_temp_sp > (uint16_t)THIGH))) {
			for (brojac = (uint8_t)0; brojac < (uint8_t)3; brojac++) {
				//printf("LED sija\n");
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
				//printf("LED sija\n");
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


static void timer_seg7(TimerHandle_t tm) {

	 static uint8_t dd;
	

	if (get_LED_BAR(2, &dd) != 0)  //uzimamo podatke o pritisnutim tasterima sa treceg stupca
	{
		printf("Greska prilikom ocitavanja led bara\n");
	}
	
	if (dd == 1 && promenljiva == 0) {
		promenljiva = 1;
		dd = 0;
		//printf("usao u if dd == 1 \n");
		if (xSemaphoreGive(seg7_un, portMAX_DELAY) != pdTRUE) {
			printf("Neuspesno davanje semafora seg7_un\n");
		}
		else printf("dat je semafor seg7_un\n");
		
	}
	else if (dd == 2 && promenljiva2 == 0) {
		promenljiva2 = 1;
		dd = 0;
		//printf("usao u if dd == 2 \n");
		if (xSemaphoreGive(seg7_sp, portMAX_DELAY) != pdTRUE) {
			printf("Neuspesno davanje semafora seg7_sp\n");
		}
		else printf("dat je semafor seg7_sp\n");
		
	}
	//else printf("nije stisnut nijedan taster\n");
}

static void mux_seg7(void* pvParameters) {
	//uint8_t d;
	uint16_t ku;
	uint16_t ou;
	uint16_t desetica_un_t;
	uint16_t jedinica_un_t;
	uint16_t desetica_un_o;
	uint16_t jedinica_un_o;
	//uint8_t br;
	for (;;) {
		printf("usao u for sp\n");
		//vTaskDelay(pdMS_TO_TICKS(100));
		/*if (xQueueReceive(seg7, &d, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda seg7\n");
		}
		else printf("primio d\n");*/
		if (xQueueReceive(kalibrisana_unutrasnja, &ku, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost\n");
		}
		else printf("primio ku\n");

		if (xQueueReceive(otpornost_u, &ou, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost\n");
		}
		else printf("primio ou\n");

		
		

		printf("ku - temp unutrasnja %d\n", ku);
		printf("ou - otpornost unutrasnja %d\n", ou);
		desetica_un_t = ku / (uint16_t)10;
		//printf("desetica kod unutrasnje temperature %d\n", desetica_un_t);
		jedinica_un_t = ku % (uint16_t)10;
		//printf("jedinica kod unutrasnje temperature %d\n", jedinica_un_t);
		desetica_un_o = ou / (uint16_t)10;
		//printf("desetica kod unutrasnje otpornosti %d\n", desetica_un_o);
		jedinica_un_o = ou % (uint16_t)10;
		//printf("jedinica kod unutrasnje otpornosti %d\n", jedinica_un_o);

		promenljiva = 0;

	if (xSemaphoreTake(seg7_un, portMAX_DELAY) != pdTRUE) {
		printf("Neuspesno TAKE semafora seg7_un\n");
	} 
	else {
		printf("uzeo semafor seg7_un\n");
		/*for (;;) {
			//vTaskDelay(pdMS_TO_TICKS(100));

			/*if (get_LED_BAR(2, &d) != 0)  //uzimamo podatke o pritisnutim tasterima sa treceg stupca
			{
				printf("Greska prilikom ocitavanja led bara\n");
			}
			else printf("ocitao led bar\n");*/

		//	if (d == (uint8_t)1) {  //uzimaju se podaci za unutrasnju temperaturu sa prvog tastera od gore
				//ispis temperature
		printf("stisnut taster prvi\n");
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
		//d = 0;
	}

	//}
	
	//}

	

	
	}
	ou = 0;
	ku = 0;

}

static void mux_seg7_sp(void* pvParameters) {
	uint16_t desetica_sp_t;
	uint16_t jedinica_sp_t;
	uint16_t desetica_sp_o;
	uint16_t jedinica_sp_o;
	uint16_t os;
	uint16_t ks;
	//uint8_t d;

	for (;;) {
		printf("usao u for sp\n");
		/*if (xQueueReceive(seg7, &d, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda seg7\n");
		}
		else printf("primio d\n");*/
		if (xQueueReceive(kalibrisana_spoljasnja, &ks, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost\n");
		}
		else printf("primio ks\n");

		if (xQueueReceive(otpornost_s, &os, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem iz reda max_otpornost\n");
		}
		else printf("primio os\n");


		printf("os - otpornost spoljasnja %d\n", os);
		printf("ks - temp spoljasnja %d\n", ks);

		desetica_sp_t = ks / (uint16_t)10;
		//printf("desetica kod spoljasnje temperature %d\n", desetica_sp_t);
		jedinica_sp_t = ks % (uint16_t)10;
		//printf("jedinica kod spoljasnje temperature %d\n", jedinica_sp_t);
		desetica_sp_o = os / (uint16_t)10;
		//printf("desetica kod spoljasnje otpornosti %d\n", desetica_sp_o);
		jedinica_sp_o = os % (uint16_t)10;
		//printf("jedinica kod spoljasnje otpornosti %d\n", jedinica_sp_o);
		
		promenljiva2 = 0;

	if (xSemaphoreTake(seg7_sp, portMAX_DELAY) != pdTRUE) {
		printf("Neuspesno TAKE semafora seg7_sp\n");
	}
	else {
		printf("uzeo semafor seg7_sp\n");
		printf("stisnut taster drugi\n");
		//for (;;) {

			/*if (get_LED_BAR(2, &d) != 0)  //uzimamo podatke o pritisnutim tasterima sa treceg stupca
			{
				printf("Greska prilikom ocitavanja led bara\n");
			}*/

			//if (d == (uint8_t)2) {  //uzimaju se podaci za spoljasnju temperaturu ako je pritisnut drugi taster od dole
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
		//d = 0;
	}
		//}

	}
	ks = 0;
	os = 0;
	//}
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

	//	init_serial_uplink(COM_CH_2); // inicijalizacija serijske TX na kanalu 2
	//	init_serial_downlink(COM_CH_2);// inicijalizacija serijske RX na kanalu 2

	if (init_7seg_comm() != 0) {
		printf("Neuspesna inicijalizacija \n");
	}


	/* ON INPUT CHANGE INTERRUPT HANDLER */
	//vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);
	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	/* Create LED interrapt semaphore */
	//LED_INT_BinarySemaphore = xSemaphoreCreateBinary();
	//LED_INT_BinarySemaphore2 = xSemaphoreCreateBinary();

	/* Create TBE semaphores - serial transmit comm */
	RXC_BS_0 = xSemaphoreCreateBinary();
	RXC_BS_1 = xSemaphoreCreateBinary();
	seg7_un = xSemaphoreCreateBinary();
	seg7_sp = xSemaphoreCreateBinary();
	//	RXC_BS_2 = xSemaphoreCreateBinary();

	/* SERIAL RECEIVER TASK */
	status = xTaskCreate(SerialReceiveTask_0, "SR0", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja SerialReceive_Task0\n");
	}
	/* SERIAL RECEIVER TASK */
	status = xTaskCreate(SerialReceiveTask_1, "SR1", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja Serialreceive_Task1\n");
	}
	//	xTaskCreate(SerialReceiveTask_2, "SR2", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL);

	/* SERIAL TRANSMITTER TASK */
	status = xTaskCreate(SerialSend_Task0, "ST0", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja SerialSend_Task0\n");
	}
	/*status = xTaskCreate(SerialSend_Task1, "STx1", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja SerialSend_Task0\n");
	}*/
	/*status = xTaskCreate(SerialSend_Task2, "STx2", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI + 1, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja SerialSend_Task0\n");
	}*/
	status = xTaskCreate(prosecna_temp_un, "prosecna_temp_un", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska prosecna_temp_un\n");
	}
	status = xTaskCreate(prosecna_temp_sp, "prosecna_temp_sp", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska prosecna_temp_sp\n");
	}
	status = xTaskCreate(kalibracija, "kalibracija", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska kalibracija_sp\n");
	}


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

	/*seg7 = xQueueCreate(10, sizeof(uint16_t));
	if (seg7 == NULL)
	{
		printf("Greska prilikom kreiranja reda seg7\n");
	}*/

	status = xTaskCreate(mux_seg7, "mux_seg7", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska mux_seg7\n");
	}
	status = xTaskCreate(mux_seg7_sp, "mux_seg7_sp", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska mux_seg7\n");
	}

	// Timers
	tH1 = xTimerCreate(
		"test timer",
		pdMS_TO_TICKS(100),
		pdTRUE,
		0,
		timer_seg7
	);
	xTimerStart(tH1, 0);

	/*tH2 = xTimerCreate(
		"test timer",
		pdMS_TO_TICKS(110),
		pdTRUE,
		0,
		timer_seg7_sp
	);
	xTimerStart(tH2, 0);*/

	r_point = 0;
	r_point1 = 0;
	vTaskStartScheduler();

	for (;;) {}
}
