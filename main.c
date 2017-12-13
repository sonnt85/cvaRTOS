#include <mega8.h>
#include <stdio.h>
#include <delay.h>  

                        
#include "kernel.h"               


void _idle_task()
{                      
	for(;;)
	{
    // do something ...   
    }   
}
                         
   
void task_1(void )
{	
    for(;;)
    {   
    PORTB.1=~PORTB.1;
	_task_delay(1); 
 //   delay_ms(10);
    }
}
       
void task_2(void )
{ 
    for(;;)
    {
    PORTB.2=~PORTB.2;
	_task_delay(2);  
   //  delay_ms(20);
    }
}

void task_3(void )
{ 
    for(;;)
    {
    PORTB.3=~PORTB.3;
	//_task_delay(10);  
     delay_ms(10);
    }
}

void main(void)
{
PORTB=0x00;
DDRB=0xFF;   
                                
_init_rtos(); 
_create_task(0,_idle_task, 2,10, TASK_READY); 		//always init this task first
_create_task(1,task_1,16,100,TASK_READY);
_create_task(2,task_2,16,100,TASK_READY);
_create_task(3,task_3,16,100,TASK_READY);
_run();
  
}                 
