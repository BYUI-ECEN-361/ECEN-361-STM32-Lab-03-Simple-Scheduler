/**********************************************
 * Global Variables and types
 * ECEN-361 Lab-3  Write a simple scheduler
 **********************************************/
#include <stdbool.h>
#include <stdio.h>
#define MAXTASKS 4
// DIAGNOSTIC is a simple flag that kicks in extra statements
// if we need to see more info during run time
#define DIAGNOSTIC  false

typedef void (*task_cb)();
int last_runtime;
int num_tasks = 0;
uint32_t switch_status = 0;

int16_t Task_D1_Count = 0;
int16_t Task_D4_Count = 0;

char uart_buf[50];
int uart_buf_len;
uint16_t timer_val;
bool first_idle = true;

UART_HandleTypeDef huart2;

typedef struct
    {
    int period;
    int remaining_time;
    bool suspended;
    task_cb callback;
    } taskControlBlock;


	/** Here is the queue -- well, list of task control blocks **/

taskControlBlock list_of_tasks_to_do[MAXTASKS];


/**  declarations */
void Scheduler_Init()
	{ last_runtime = uwTick; }

void Scheduler_Toggle_Suspend(int TCB_number)
	{
	/* incoming is the task_control_block number */
	list_of_tasks_to_do[TCB_number].suspended = !list_of_tasks_to_do[TCB_number].suspended;
	}

/* Function to add tasks to the queue */

void Scheduler_StartTask(int period, task_cb task)
{
    if (num_tasks < MAXTASKS)
    {
        list_of_tasks_to_do[num_tasks].period = period;
        list_of_tasks_to_do[num_tasks].remaining_time = 0;
        list_of_tasks_to_do[num_tasks].suspended = false;
        list_of_tasks_to_do[num_tasks].callback = task;
        num_tasks++;
    }
}


/* Function that will be run when no other functions are running. */
void idle_process();
/* Go through the task list and run any tasks that need to be run. */




/*************** HERE ARE THE SEPARATE TASKS!   *******************/

void D1_task()
    { TASK_D1_LED_TOGGLE
		MultiFunctionShield_Single_Digit_Display(4,Task_D1_Count++);
    	first_idle = true;

    }

void D4_task()
    { TASK_D4_LED_TOGGLE
		MultiFunctionShield_Single_Digit_Display(1,Task_D4_Count++);
    	first_idle = true;
	  }

void uartout_task()
    {
	/*
    UART_write(uart, "uartout_task: ", 13);
    print_out_the_systick_count();
    */
    }

void idle_process()
    {
    /*
     * This function can perform some low-priority task while the scheduler has nothing to do
     *  Example here:  Uart transmitting the systick value
     *
     *  It should return before the idle period (measured in ms) has expired.
     */
	if(first_idle)
	{
	//get systick timer value
	timer_val = HAL_GetTick();
	//show systick value
	uart_buf_len = sprintf(uart_buf, "Entered IDLE at %u\r\n", timer_val);
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, uart_buf_len, 100);
	//make sure it only happens on first idle after a task
	first_idle = false;
	}

    }



