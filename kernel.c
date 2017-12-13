#include <string.h>
#include "kernel.h"
#include "mem.h"
#include "stdio.h"
volatile unsigned char _index;
volatile unsigned char * _mem_ptr;
volatile unsigned char _task_count;
volatile unsigned char _current_task,_time_slide;
volatile unsigned char _task_status[MAX_TASK];
volatile unsigned int  _task_system_stack[MAX_TASK];
volatile unsigned int  _task_data_stack[MAX_TASK];
volatile unsigned int  _task_tick_count[MAX_TASK - 1];
volatile unsigned char _msg_wr_index[MAX_TASK - 1], _msg_rd_index[MAX_TASK - 1], _msg_counter[MAX_TASK -1];
volatile unsigned char _queue_task[MAX_TASK - 1][QUEUE_SIZE];// task0 has not queue
volatile unsigned char * _sem_ptr[MAX_TASK - 1]; // task0 has not sem
volatile unsigned char _lweven[MAX_TASK - 1];
volatile unsigned char * _task_mark_ds[MAX_TASK];
volatile unsigned char * _task_mark_hs[MAX_TASK];
#asm
    .MACRO _SET_TOIE0
        PUSH R17
        PUSH R18
        IN   R17, SREG
        PUSH R17

        IN   R18,0x39
        LDI  R17,LOW(1)
        OR   R17,R18
        OUT  0x39,R17

        POP  R17
        OUT  SREG,R17
        POP  R18
        POP  R17
    .ENDM
    .MACRO _CLEAR_TOIE0
        PUSH R17
        PUSH R18
        IN   R17, SREG
        PUSH R17

        IN   R18,0x39
        LDI  R17,LOW(~1)
        AND  R17,R18
        OUT  0x39,R17
        POP  R17
        OUT  SREG,R17

        POP  R18
        POP  R17
    .ENDM
#endasm

#define  CLEAR_TOIE0() #asm("_CLEAR_TOIE0")
#define  SET_TOIE0()   #asm("_SET_TOIE0")

#define _save_context() \
    #asm ("  SEI                                ");\
    #asm ("  PUSH R0                            ");\
    #asm ("  PUSH R1                            ");\
    #asm ("  PUSH R2                            ");\
    #asm ("  PUSH R3                            ");\
    #asm ("  PUSH R4                            ");\
    #asm ("  PUSH R5                            ");\
    #asm ("  PUSH R6                            ");\
    #asm ("  PUSH R7                            ");\
    #asm ("  PUSH R8                            ");\
    #asm ("  PUSH R9                            ");\
    #asm ("  PUSH R10                           ");\
    #asm ("  PUSH R11                           ");\
    #asm ("  PUSH R12                           ");\
    #asm ("  PUSH R13                           ");\
    #asm ("  PUSH R14                           ");\
    #asm ("  PUSH R15                           ");\
    #asm ("  PUSH R16                           ");\
    #asm ("  PUSH R17                           ");\
    #asm ("  PUSH R18                           ");\
    #asm ("  PUSH R19                           ");\
    #asm ("  PUSH R20                           ");\
    #asm ("  PUSH R21                           ");\
    #asm ("  PUSH R22                           ");\
    #asm ("  PUSH R23                           ");\
    #asm ("  PUSH R24                           ");\
    #asm ("  PUSH R25                           ");\
    #asm ("  PUSH R26                           ");\
    #asm ("  PUSH R27                           ");\
    #asm ("  PUSH R30                           ");\
    #asm ("  PUSH R31                           ");\
    #asm ("  IN   R31,SREG                      ");\
    #asm ("  PUSH R31                           ");\           //using all 31 reg
    #asm ("  LDS  R30, __current_task           ");\           //load direct from ram R30 = _current_task
    #asm ("  CLR  R31                           ");\
    #asm ("  LSL  R30                           ");\           //logiccal shift left = 2*&_current_task
    #asm ("  LDI  R26,LOW(__task_system_stack)  ");\           //load immediate R26 = _task_system_stack = const
    #asm ("  LDI  R27,HIGH(__task_system_stack) ");\           //R27R26 = &_task_system_stack[0]
    #asm ("  ADD  R26,R30                       ");\           //Add no carry
    #asm ("  ADC  R27,R31                       ");\           //X(R27R26) = &_task_system_stack[_current_task]
    #asm ("  IN   R23,SPL                       ");\
    #asm ("  IN   R24,SPH                       ");\           //R24R23 = SP = &stack
    #asm ("  ST   X+,R23                        ");\
    #asm ("  ST   X,R24                         ");\           //_task_system_stack[_current_task] = R24R23 = SP;
    #asm ("  LDI  R26,LOW(__task_data_stack)    ");\
    #asm ("  LDI  R27,HIGH(__task_data_stack)   ");\
    #asm ("  ADD  R26,R30                       ");\
    #asm ("  ADC  R27,R31                       ");\           //X(R27R26) = &_task_data_stack[_current_task]
    #asm ("  ST   X+,R28                        ");\           //store Y (using to transmit parameters for functions)
    #asm ("  ST   X,R29                         ");            //_task_data_stack[_current_task] =  Y(R29R28)

#define _restore_context()\
    #asm (" LDS  R30, __current_task            ");\
    #asm (" CLR  R31                            ");\
    #asm (" LSL  R30                            ");\
    #asm (" LDI  R26,LOW(__task_data_stack)     ");\
    #asm (" LDI  R27,HIGH(__task_data_stack)    ");\
    #asm (" ADD  R26,R30                        ");\
    #asm (" ADC  R27,R31                        ");\
    #asm (" LD   R28,X+                         ");\
    #asm (" LD   R29,X                          ");\
    #asm (" LDI  R26,LOW(__task_system_stack)   ");\
    #asm (" LDI  R27,HIGH(__task_system_stack)  ");\
    #asm (" ADD  R26,R30                        ");\
    #asm (" ADC  R27,R31                        ");\
    #asm (" LD   R23,X+                         ");\
    #asm (" LD   R24,X                          ");\
    #asm (" OUT  SPL,R23                        ");\
    #asm (" OUT  SPH,R24                        ");\
    #asm (" POP  R0                             ");\
    #asm (" OUT  SREG, R0                       ");\
    #asm (" POP  R31                            ");\
    #asm (" POP  R30                            ");\
    #asm (" POP  R27                            ");\
    #asm (" POP  R26                            ");\
    #asm (" POP  R25                            ");\
    #asm (" POP  R24                            ");\
    #asm (" POP  R23                            ");\
    #asm (" POP  R22                            ");\
    #asm (" POP  R21                            ");\
    #asm (" POP  R20                            ");\
    #asm (" POP  R19                            ");\
    #asm (" POP  R18                            ");\
    #asm (" POP  R17                            ");\
    #asm (" POP  R16                            ");\
    #asm (" POP  R15                            ");\
    #asm (" POP  R14                            ");\
    #asm (" POP  R13                            ");\
    #asm (" POP  R12                            ");\
    #asm (" POP  R11                            ");\
    #asm (" POP  R10                            ");\
    #asm (" POP  R9                             ");\
    #asm (" POP  R8                             ");\
    #asm (" POP  R7                             ");\
    #asm (" POP  R6                             ");\
    #asm (" POP  R5                             ");\
    #asm (" POP  R4                             ");\
    #asm (" POP  R3                             ");\
    #asm (" POP  R2                             ");\
    #asm (" POP  R1                             ");\
    #asm (" POP  R0                             ");

void _rtos_check_comp()
{
    unsigned char _index;
    for (_index = 0; _index < _task_count - 1; _index++)                        //xu ly cac task delay(tick)
    {
        if (_task_status[_index] & TASK_WAIT_TIME)                              //check task waiting to run
        {
            if (_task_tick_count[_index]) _task_tick_count[_index]--;           // if timeout then set task to timeout state
            else
            {
//                _task_status[_index] &= (~TASK_WAIT_TIME);
                _task_status[_index] = (TASK_TIMEOUT | TASK_READY);
            }
        }

        if(_task_status[_index] & TASK_WAIT_SEMA) //process sem wait
        {
            if((*_sem_ptr[_index]) > 0)
            {
                (*_sem_ptr[_index])--;
                _task_status[_index] &= (~TASK_WAIT_TIME);
                _task_status[_index] |= TASK_READY;
            }
        };

        if(_task_status[_index] & TASK_WAIT_EVENT) //process sem wait
        {
            if(!(_lweven[_index]))
            {
                _task_status[_index] &= (~TASK_WAIT_EVENT);
                _task_status[_index] |= TASK_READY;
            }
        };
    }
}

unsigned char _check_ready_task()
{
    unsigned char _index;
    for(_index = 0; _index < (_task_count - 1); _index++)
    {
        if(++_current_task >= (_task_count - 1)) _current_task = 0;                  //start finding from task 1 to task _task_count - 1
        if(_task_status[_current_task] & TASK_READY)
        {
            break;                    //find a READY task
        }
    }

    if(_index == (_task_count - 1))                                             //neu ko thay task nao ready thi chay task idle
    {
        _current_task = _task_count  - 1;
    }

    for(_index = 0; _index < _task_count; _index++)
    {
         if(*_task_mark_hs[_index] !=  MARK_HS) while(1) printf("Overflow hwstack task %d\n", _current_task );
         if(*_task_mark_ds[_index] !=  MARK_DS) while(1) printf("Overflow dwstack task %d\n", _current_task );
    }
    return  _current_task;
}
#pragma savereg-
interrupt [TIM0_OVF] void timer0_ovf_isr()
{
    _save_context();
  //  TCNT0 = -0x4E;
    TCNT0 = -(CYCLE_TICK_US / 1000U * _MCU_CLOCK_FREQUENCY_ / 1024000UL);
    _rtos_check_comp();
    _check_ready_task();
    _restore_context();
}
#pragma savereg+

void _init_rtos()
{
//    Timer/Counter 0 initialization
//    Clock source: System Clock
//    Clock value: 7.813 kHz
    TCCR0 = 0x05;                   // 10ms
#if (\
        ((CYCLE_TICK_US / 1000U * _MCU_CLOCK_FREQUENCY_ / 1024000UL) > (0xFF + 1))  \
        || ((CYCLE_TICK_US / 1000U * _MCU_CLOCK_FREQUENCY_ / 1024000UL) <= 1)       \
    )
    #error " CYCLE_TICK_US very big, dec CYCLE_TICK_US"
#endif
    TCNT0 = -(CYCLE_TICK_US / 1000U * _MCU_CLOCK_FREQUENCY_ / 1024000UL);
    TIMSK |= (1 << TOIE0);          //inable flag interrupt overflow
    // init default state for tasks
    _task_count = 0;                // notask to run
    _current_task = 0;
    for (_index=0;_index<MAX_TASK;_index++)
      _task_status[_index] = TASK_SUSPEND;

//    init starting RAM address for allocate system RTOS stack and data stack memory
    _mem_ptr = (char*)(_SRAM_START_ + _DSTACK_SIZE_ - 1);

    #asm ("LDI R29,HIGH(__SRAM_END - __HEAP_SIZE - 20)"); // move X (cause cvavr using) to tem add to using setup system
    #asm ("LDI R28,LOW(__SRAM_END  - __HEAP_SIZE - 20)");
}

unsigned int _create_task(unsigned char tsk_no,void (*func)(void),unsigned char sys_size,unsigned char data_size,unsigned char state)
{
    unsigned int i;
    if((_SRAM_START_ - 1) > ((unsigned int)_mem_ptr - (31 + sys_size + data_size))) return 0;
    for(i = 0; i < (31 + sys_size); i++)
    {
        *(_mem_ptr - i) =  MARK_HS;
    }

    for(i = 31 + sys_size; i < data_size + 31 + sys_size; i++)
    {
        *(_mem_ptr - i) =  MARK_DS;
    }

    *(_mem_ptr--) = (unsigned int)func & 0xff;                                 //-1
    *(_mem_ptr--) = (unsigned int)func >> 8;                                   //-1

    for(i=0;i <= 27; i++)
    {
        *(_mem_ptr--) = peekb(i);
    }
    *(_mem_ptr--) = peekb(30);
    *(_mem_ptr--) = peekb(31);

    *(_mem_ptr--) = SREG | 0x80;                                               //-1
    _task_system_stack[tsk_no] = (unsigned int)(_mem_ptr);

    _mem_ptr -= (((sys_size) ? (sys_size) : DEFAULT_SYS_STACK_SIZE) - 2);           //-sys_size+2  <=> -31-sys_size

    _task_mark_hs[tsk_no] = _mem_ptr + 1;
//    *(_mem_ptr + 1) = MARK_HS;                                                          //marked section end of sys stack
    _task_data_stack[tsk_no] = (unsigned int) (_mem_ptr + 1);                       // Push ST -Y, POP ST Y

    _mem_ptr -= (data_size) ? (data_size) : DEFAULT_DATA_STACK_SIZE;                //-data_Size
    _task_mark_ds[tsk_no] = _mem_ptr + 1;
//    *(_mem_ptr + 1) = MARK_DS;                                                          //marked section end of data stack
    _task_status[tsk_no] = state;
    _task_tick_count[tsk_no] = 0;

    _msg_counter[tsk_no] = 0;
    _msg_rd_index[tsk_no] = 0;
    _msg_wr_index[tsk_no] = 0;

    tsk_no++;                                                                   // increment this value for task_count
    if (_task_count < tsk_no) _task_count = tsk_no;

    return 31+(unsigned int)sys_size + data_size;
}

void _task_switch()
{
    CLEAR_TOIE0();
    _save_context();
    //find a ready task
    _rtos_check_comp();
    _check_ready_task();
    _restore_context();
    SET_TOIE0();
    #asm ("RETI"); // disable     #asm ("ADIW R28, 1");
}

void _task_switch_to(unsigned char taskno)
{
    CLEAR_TOIE0();
    #asm ("ADIW R28, 1");
    _save_context();
    #asm ("SBIW R28, 1");
    _current_task  = taskno;
    _task_status[taskno] |= TASK_READY; // mask 4 bit firt for semaphore
    _rtos_check_comp();
    _restore_context();
    SET_TOIE0();
    #asm ("RETI");
}

void _task_sw_int()
{
    _task_switch();
}
void _task_delay(unsigned int ms)
{
    CLEAR_TOIE0();
    _task_tick_count[_current_task] = ms  / CYCLE_TICK_MS;
    _task_status[_current_task] &= (~TASK_READY);
    _task_status[_current_task] |=  TASK_WAIT_TIME;
    _task_switch();
}

void _run()
{
    _current_task = 0;
    TCNT0 = -(CYCLE_TICK_US / 1000U * _MCU_CLOCK_FREQUENCY_ / 1024000UL);
    TIFR |= 1; //clear TOVT0 (datasheet)
    _restore_context();
    #asm("SEI")
}

unsigned char _msg_poll()
{
    return _msg_counter[_current_task];
}

char _msg_read()
{
    char data;
    data = _queue_task[_current_task][_msg_rd_index[_current_task]];
    if((_msg_rd_index[_current_task]++) == QUEUE_SIZE)_msg_rd_index[_current_task] = 0;
    _msg_counter[_current_task]--;
    return data;
}

char _msg_send(unsigned char tskno, char data)
{
    _queue_task[_current_task][_msg_wr_index[tskno]] = data;
     if((_msg_wr_index[tskno]++) == QUEUE_SIZE)_msg_wr_index[tskno] = 0;
    if((_msg_counter[tskno]++) == QUEUE_SIZE)_msg_counter[tskno] = 0;
    return _msg_counter[tskno];
}

void _msg_flush()
{
    _msg_counter[_current_task]  = 0;
    _msg_rd_index[_current_task] = 0;
    _msg_wr_index[_current_task] = 0;
}

char _sem_wait(unsigned int ticks, char *sem)
{
    CLEAR_TOIE0();
    _task_status[_current_task] &= (~TASK_READY);
    _task_status[_current_task] |= TASK_WAIT_SEMA;
    _sem_ptr[_current_task ] = sem;
    if (ticks)
    {
        _task_tick_count[_current_task] = ticks / CYCLE_TICK_MS;
        _task_status[_current_task] |= TASK_WAIT_TIME;
    }
//    while((*sem <= 0) &&  (!(_task_status[_current_task] & TASK_READY)))
//    #asm ("ADIW R28, 4");
    _task_sw_int();
    SET_TOIE0();
    return  (_task_status[_current_task] & TASK_TIMEOUT) ? TASK_TIMEOUT: 0;
}

void _sem_signal(char *sem)
{
    CLEAR_TOIE0();
    if(*sem != 127)((*sem)++);
    SET_TOIE0();
}

void _task_await(char *expre)
{
    CLEAR_TOIE0();
    while(!(*expre))_task_sw_int();
    SET_TOIE0();
}

void _task_wait_lwevent(unsigned char ticks)
{
    CLEAR_TOIE0();
    _task_status[_current_task] &= (~TASK_READY);
    _task_status[_current_task] |= TASK_WAIT_EVENT;
    if (ticks)
    {
        _task_tick_count[_current_task] = ticks / CYCLE_TICK_MS;
        _task_status[_current_task] |= TASK_WAIT_TIME;
    }
    _task_sw_int();
    SET_TOIE0();
}

void _task_set_lwevent(char mark, unsigned char tskno)
{
    CLEAR_TOIE0();
    _lweven[tskno] &= (~mark); // clear
    _rtos_check_comp();
//    _task_sw_int();
    SET_TOIE0();
}

void _task_clear_lwevent(char mark, unsigned char tskno)
{
    CLEAR_TOIE0();
    _lweven[tskno] |= mark; // set
    _rtos_check_comp();
//    _task_sw_int();
    SET_TOIE0();
}

unsigned char _query_task()
{
    return _task_status[_current_task] & (~TASK_TIMEOUT);
}

void _set_task_status(unsigned char state)
{
    CLEAR_TOIE0();
    _task_status[_current_task] = state;
//    if(_task_status[_current_task] & TASK_READY) _task_sw_int();
    SET_TOIE0();
}

char _set_taskn_status(unsigned char taskno, unsigned char state)
{
    if (taskno < _task_count)
    {
        CLEAR_TOIE0();
        _task_status[taskno] = state;
        if((taskno == _current_task) && (_task_status[_current_task] & TASK_READY)) _task_sw_int();
        SET_TOIE0();
        return 0;
    } return 1;
}
