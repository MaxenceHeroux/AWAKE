#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define PWM_PIN1 1
#define PWM_PIN2 26
#define PWM_PIN3 23

int main (void)
{
  int bright ;

  printf ("Raspberry Pi wiringPi PWM test program\n") ;
  if (wiringPiSetup () == -1) exit (1);

  pinMode (PWM_PIN1, PWM_OUTPUT) ;
  pinMode (PWM_PIN2, PWM_OUTPUT) ;
  pinMode (PWM_PIN3, PWM_OUTPUT) ;

  for (;;){
    for (bright = 0 ; bright < 1024 ; ++bright){
      pwmWrite (PWM_PIN1, bright) ;
      pwmWrite (PWM_PIN2, bright) ;
      pwmWrite (PWM_PIN3, bright) ;
      delay (1) ;
    }

    for (bright = 1023 ; bright >= 0 ; --bright){
      pwmWrite (PWM_PIN1, bright) ;
      pwmWrite (PWM_PIN2, bright) ;
      pwmWrite (PWM_PIN3, bright) ;
      delay (1) ;
    }
  }

  return 0 ;
}
