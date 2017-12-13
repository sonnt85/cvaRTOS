/*
 * RTOS for Codevision avr
 * Author: thanhson.rf@gmail.com
 * 
 */
#ifndef _kenel_h
#define _kenel_h
#include <mega8.h>
#define    MAX_TASK                    (4)
#define    QUEUE_SIZE                  (5)

#define    DEFAULT_SYS_STACK_SIZE      (10)
#define    DEFAULT_DATA_STACK_SIZE     (20)
#define    CYCLE_TICK_US               (1000UL)
#define    CYCLE_TICK_MS               (CYCLE_TICK_US / 1000U)

#define    TASK_SUSPEND                (0)
#define    TASK_READY                  (1)
#define    TASK_WAIT_TIME              (2)
#define    TASK_WAIT_SEMA              (4)
#define    TASK_TIMEOUT                (8)
#define    TASK_WAIT_EVENT             (0X10)

#define    MARK_HS                     'H'
#define    MARK_DS                     'D'

extern volatile unsigned char _current_task,time_slide;
void _init_rtos();
unsigned int _create_task(unsigned char tsk_no,void (*func)(void),unsigned char sys_size,unsigned char data_size,unsigned char state);
void _run();
void _task_delay(unsigned int ms);
void _task_switch();
void _task_switch_to(unsigned char taskno);

unsigned char _msg_poll();
char _msg_read();
char _msg_send(unsigned char tskno, char data);
void _msg_flush();

char _sem_wait(unsigned int ticks, char *sem);
void _sem_signal(char *sem);
void _task_await(char *expre);

void _task_clear_lwevent(char mark, unsigned char tskno);
void _task_set_lwevent(char mark, unsigned char tskno);
void _task_wait_lwevent(unsigned char ticks);

unsigned char _query_task();
void _set_task_status(unsigned char state);
char _set_taskn_status(unsigned char taskno, unsigned char state);
#endif
