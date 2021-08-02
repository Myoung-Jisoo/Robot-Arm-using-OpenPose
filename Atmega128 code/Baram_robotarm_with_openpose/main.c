#include "mcu_init.h"

int cnt = 0;
int adc_array[8] = {0, };
int i = 0;
float servo1 = .080;
float servo2 = .075;
int duty1, duty2;
// duty 2.5~12.5

int num1 = 0;
int num2 = 0;
int pnum1 = 0;
int pnum2 = 0;
char a[9] = {0,};
int flag;
int number = 0;
int sign = 0;
// UART 통신

ISR(TIMER2_OVF_vect)
{
	cnt++;
	TCNT0 = 131;   // 4ms
	DDRA = 0xff;
	
	a[0] = UART1_Receive();
	
	if(a[0] == 's'){
		for(int i = 1; i < 9; i++){a[i] = UART1_Receive();}
			
		for(int i = 0; a[i] != 'e'; i++) {UART1_Transmit(a[i]);}
		UART1_Transmit(13);
		
		int j = 1;
		
		for( ; a[j] != ','; j++) { Receive_num1(a[j]); }
		if(sign == 1) num1 = -num1;
		sign = 0; j++;
		
		for( ; a[j] != 'e'; j++) { Receive_num2(a[j]); }

		if(sign == 1) num2 = -num2;
		sign = 0;
		
		UART1_TransNum(num1);
		UART1_TransNum(num2);
		UART1_Transmit(13);
		
		if((num1 >= 0) && (num1 <= 150)) servo1 = 0.080 + 0.045 * ((double)num1 / 150);
		else if(num1 > 150) servo1 = 0.080 + 0.045;
		
		if((num2 >= -90) && (num2 <= 90)) servo2 = 0.075 + 0.045 * ((double)num2 / 90);
		else if(num2 > 90) servo2 = 0.075 + 0.045;
		else if(num2 < -90) servo2 = 0.075 - 0.045;
		
		
		
		num1 = 0; num2 = 0; flag = 0;
	}
	
	if(abs(OCR3C - (1250 * servo1)) > 1) {
		if(OCR3C < (1250 * servo1)) OCR3C += 1;
		else if(OCR3C > (1250 * servo1)) OCR3C -= 1;
	}
	if(abs(OCR3B - (1250 * servo2)) > 1) {
		if(OCR3B < (1250 * servo2)) OCR3B += 1;
		else if(OCR3B > (1250 * servo2)) OCR3B -= 1;
	}
}


void Receive_num1(char a) // 첫번째 수를 입력받는다.
{
	num1 *= 10;
	number = a - 48;
	if(a == '-') sign = 1;
	if((number >= 0) && (number <= 9)) num1 += number;
}

void Receive_num2(char a) // 두번째 수를 입력받는다.
{
	num2 *= 10;
	number = a - 48;
	if(a == '-') sign = 1;
	if((number >= 0) && (number <= 9)) num2 += number;
}

ISR(INT0_vect)
{
	servo1 = .080;
}

ISR(INT1_vect)
{
	servo2 = .075;
}

int main(void)
{
	UART1_INIT();
	Timer2_INIT();
	BUTTON_INIT();
	SERVO_MOTOR_INIT();
	ADC_INIT();
	
	sei();
	
	while(1){}
}