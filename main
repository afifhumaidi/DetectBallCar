/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 12/10/2019
Author  : 
Company : 
Comments: 


Chip type               : ATmega328P
Program type            : Application
AVR Core Clock frequency: 16.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*******************************************************/

#include <mega328p.h>

#include <delay.h>

// Declare your global variables here
float error_p, error_i, error_d, last_error=0;
float p, i, d, pidat;
float error_p2, error_i2, error_d2, last_error2=0;
float p2, i2, d2, pidat2;
unsigned char data[10];
const float kp = 8, ki = 50, kd = 0.1, ts=0.01;
const float kp2 = 8, ki2 = 50, kd2 = 0.01;
static int flag, jarak, posisi;
    
// Standard Input/Output functions
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (1<<ADLAR))

// Read the 8 most significant bits
// of the AD conversion result
unsigned char read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCH;
}

void pid(int set_point){
    
	error_p = set_point - 30;
	error_i += error_p * ts;
	if(error_i > 3)
	error_i = 3;
	else if(error_i < -3)
	error_i = -3;
    //ftoa(error_i, 2, data);
    //printf("%s ", data);
	error_d = (error_p - last_error)/ts;
	last_error = error_p;
	                          
	p = error_p * kp;
	i = error_i * ki;
	d = error_d * kd;
    
	pidat = p + i + d;
    //ftoa(pidat, 2, data);
    //printf("%s ", data);
    pidat = abs(pidat);  
    //ftoa(pidat, 2, data);
    //printf("%s\n", data);
    
	if(pidat > 200)
	pidat = 200;
	else if(pidat < 0)
	pidat = 0;
}

void pid2(int set_point2){
    
	error_p2 = set_point2 - 200;
	error_i2 += error_p2 * ts;
	if(error_i2 > 4)
	error_i2 = 4;
	else if(error_i2 < -4)
	error_i2 = -4;
    ftoa(error_i2, 2, data);
    printf("%s ", data);
	error_d2 = (error_p2 - last_error2)/ts;
	last_error2 = error_p2;
	                          
	p2 = error_p2 * kp2;
	i2 = error_i2 * ki2;
	d2 = error_d2 * kd2;
    
	pidat2 = p2 + i2 + d2;
    ftoa(pidat2, 2, data);
    printf("%s ", data);
    pidat2 = abs(pidat2);  
    ftoa(pidat2, 2, data);
    printf("%s\n", data);
    
	if(pidat2 > 200)
	pidat2 = 200;
	else if(pidat2 < 0)
	pidat2 = 0;
}

void pwm_update(int valPwm){
    //printf("%d\n", (int)pidat);

    OCR2A=valPwm;
    OCR2B=valPwm;
}

//2 kiri 3 kanan 0 mundur 1 maju
void pwm_dir(char arah){
	char m1=2, m2=1; 
    int i;
	if(arah == 0){
		m1=2; m2=4;
	}
	else if(arah == 1){
		m1=3; m2=1;
	}
    else if(arah == 2){
        m1=2; m2=1;
    }
    else if(arah == 3){
        m1=3; m2=4;
    }
	
	for(i=7; i>=0; i--){
		if((i==m1)|(i==m2))
		PORTB |= (1 << PORTB0);
		else
		PORTB &= ~(1 << PORTB0);
		
		PORTD |= (1 << PORTD4);
		PORTD &= ~(1 << PORTD4);
	}
	PORTB |= (1 << PORTB4);
	PORTB &= ~(1 << PORTB4);
}

void findball(){
	pwm_dir(3);
	
	pwm_update(200);
}

void calculateDist(){
	if(jarak>=30)
        pwm_dir(1);
    else
		pwm_dir(0);
      
    pid(jarak);
    pwm_update((int)pidat);
}

void calculateXpos(){
	if(posisi>=200)
		pwm_dir(3);
    else
		pwm_dir(2);
        
    pid2(posisi);
    pwm_update((int)pidat2);
}

void main(void)
{
// Declare your local variables here
static int counter = 0, EnDist = 0;
// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=Out Bit3=Out Bit2=In Bit1=In Bit0=Out 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (1<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (1<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=0 Bit3=0 Bit2=T Bit1=T Bit0=0 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=In Bit5=In Bit4=Out Bit3=Out Bit2=In Bit1=In Bit0=In 
DDRD=(1<<DDD7) | (0<<DDD6) | (0<<DDD5) | (1<<DDD4) | (1<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=0 Bit6=T Bit5=T Bit4=0 Bit3=0 Bit2=T Bit1=T Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 2000.000 kHz
// Mode: Fast PWM top=0xFF
// OC2A output: Non-Inverted PWM
// OC2B output: Non-Inverted PWM
// Timer Period: 0.128 ms
// Output Pulse(s):
// OC2A Period: 0.128 ms Width: 0 us
// OC2B Period: 0.128 ms Width: 0 us
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(1<<COM2A1) | (0<<COM2A0) | (1<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (1<<WGM20);
TCCR2B=(0<<WGM22) | (0<<CS22) | (1<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2A=0;
OCR2B=0;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: Off
EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EIMSK=(0<<INT1) | (0<<INT0);
PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART0 Mode: Asynchronous
// USART Baud Rate: 9600
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x67;

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
// Digital input buffer on AIN0: On
// Digital input buffer on AIN1: On
DIDR1=(0<<AIN0D) | (0<<AIN1D);

// ADC initialization
// ADC Clock frequency: 125.000 kHz
// ADC Voltage Reference: AREF pin
// ADC Auto Trigger Source: Free Running
// Only the 8 most significant bits of
// the AD conversion result are used
// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
// ADC4: On, ADC5: On
DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

while (1)
      {        
      /*
      // Place your code here
      scanf("#%d %d %d", &flag, &jarak, &posisi); 
      
      if(jarak>=30)
        pwm_dir(1);
      else
        pwm_dir(0);
      
      pid(jarak);
      pwm_update();
      
      scanf("#%d %d %d", &flag, &jarak, &posisi);
      if(posisi>=200)
        pwm_dir(3);
      else
        pwm_dir(2);
        
      pid2(posisi);
      pwm_update2();
      */
      
      scanf("#%d %d %d", &flag, &jarak, &posisi);
      if(flag){
        if((posisi >= 180) && (posisi <= 220)){
            counter++;
            if(counter > 10)
                EnDist = 1;
        }
        else{
            EnDist = 0;
            counter = 0;
            calculateXpos();
        }
        if(EnDist){
            calculateDist();
        }
      }
      else
        findball();
      
      
      //scanf("#%d %d %d", &flag, &jarak, &posisi);
      //calculateXpos();
      //calculateDist();
      
      delay_ms(10);
      
      }
}
