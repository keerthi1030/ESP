#include<avr/io.h>
#include <util/delay.h>
#include<avr/interrupt.h>
#define ENGINE PD2
#define OVER_STEER PB2
#define ENGINE_LED PD7
#define UNDER_STEER PB1
#define SET_BIT(PORT,BIT)  PORT |= (1<<BIT)
#define CLR_BIT(PORT,BIT)  PORT &= ~(1<<BIT)
#define adcpin0 0
#define adcpin1 1
#define adcpin2 2
#define adcpin3 3
struct
{
volatile unsigned int ENG_FLAG:1;
volatile unsigned int ESC_FLAG:1;
}FLAG_BIT;
uint16_t vehicle_Speed;
  uint16_t Steering_Angle;
  uint16_t Yaw_Angle;
  int16_t difference_Angle;
  uint16_t col_sensor;
  uint16_t ADC_value;
  uint8_t count;
  uint16_t speed_read(uint8_t adc)
{
  ADMUX = adc;
  ADMUX |= (1<<REFS0);
  ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADSC);
 ADCSRA |= ((1<<ADPS1)|(1<<ADPS2));
  while(ADCSRA&(1<<ADSC));
    return(ADC);
}
 
uint16_t steer_read(uint8_t adc)
{
  ADMUX = adc;
  ADMUX |= (1<<REFS0);
  ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADSC);
  ADCSRA |= ((1<<ADPS1)|(1<<ADPS2));
  while(ADCSRA&(1<<ADSC));
    return(ADC);
}
 
uint16_t gyro_read(uint8_t adc)
{
  ADMUX = adc;
  ADMUX |= (1<<REFS0);
  ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADSC);
  ADCSRA |= ((1<<ADPS1)|(1<<ADPS2));
  while(ADCSRA&(1<<ADSC));
    return(ADC);
}
int main ()
{
    SET_BIT(DDRD,PD7);
    CLR_BIT(DDRD,PD2);
    SET_BIT(PORTD,PD2);
    SET_BIT(DDRB,PB2);
    SET_BIT(DDRB,PB1);
 
    EICRA |= (1<<ISC00);
    EIMSK |= (1<<INT0);
    SREG |= (1<<7);
    while (1)
    {
         if (FLAG_BIT.ENG_FLAG==1)
         {
             SET_BIT(PORTD,PD7);
             _delay_ms(1000);
            vehicle_Speed = speed_read(adcpin1);
            vehicle_Speed = vehicle_Speed*0.234375;
        if(vehicle_Speed > 20)
        {
 
              Steering_Angle = steer_read(adcpin2);
              Yaw_Angle = gyro_read(adcpin3);
              Steering_Angle = Steering_Angle*0.351625;
              Yaw_Angle = Yaw_Angle*0.351625;
              difference_Angle = (Steering_Angle-Yaw_Angle);
 
              if((difference_Angle)>20)
              {
                  PORTB|=(1<<OVER_STEER);
                  PORTB&=~(1<<UNDER_STEER);
              }
 
              else if((difference_Angle)<-20)
              {
                  PORTB|=(1<<UNDER_STEER);
                  PORTB&=~(1<<OVER_STEER);
              }
        }
              else
              {
                  PORTB&=~(1<<UNDER_STEER);
                  PORTB&=~(1<<OVER_STEER);
 
              }
 
 
 
         }
         else
         {
                CLR_BIT(PORTD,PD7);
                CLR_BIT(PORTB,PB1);
                CLR_BIT(PORTB,PB2);
         }
    }
}
ISR (INT0_vect)
{
if(FLAG_BIT.ENG_FLAG == 0)
{
    FLAG_BIT.ENG_FLAG = 1;
}
else{
    FLAG_BIT.ENG_FLAG = 0;
}
}
