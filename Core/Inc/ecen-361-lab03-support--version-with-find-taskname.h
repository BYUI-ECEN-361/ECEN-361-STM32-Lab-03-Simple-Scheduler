/**********************************************
 * Global Variables and types
 * ECEN-361 Lab-3  Write a simple scheduler
 **********************************************/
#include <stdbool.h>
#include <stdio.h>
#include "MultiFunctionShield.h"
#define MAXTASKS 4
// DIAGNOSTIC is a simple flag that kicks in extra statements
// if we need to see more info during run time
#define DIAGNOSTIC  true

typedef void (*task_cb)();
int last_runtime;
int num_tasks = 0;
uint32_t switch_status = 0;

int16_t Task_D1_Count = 0;
int16_t Task_D4_Count = 0;

typedef struct
    {
    int period;
    int remaining_time;
    int suspended;
    char* task_name;		// A unique string to identify this task control block single item
    task_cb callback;
    } taskControlBlock;


	/** Here is the queue -- well, list of task control blocks **/

taskControlBlock tasks[MAXTASKS];


/**  declarations */
void Scheduler_Init()
	{ last_runtime = uwTick; }


/* Function to add tasks to the queue */
/**
  * @brief Scheduler_StartTask
  * @param period   Integer defining how many SysTicks between starts
  * @param task     pointer to the routine (callback) to run
  * @param taskname The control buffer structure
  * @retval None
  */

void Scheduler_StartTask(int period, task_cb task, char* taskname)
{
    if (num_tasks < MAXTASKS)
    {
        tasks[num_tasks].period = period;
        tasks[num_tasks].remaining_time = 0;
        tasks[num_tasks].suspended = false;
        tasks[num_tasks].callback = task;
        tasks[num_tasks].task_name = taskname;
        num_tasks++;
    }
}

int find_task_number(char* theTaskName)
	{
	int taskcnt = 0;
	/* find the task */
	for (taskcnt=0;taskcnt < num_tasks;taskcnt++)
		{
		if (tasks[taskcnt].task_name == theTaskName)
		return taskcnt;
		}
	return 0;
	}

void Scheduler_Toggle_Suspend(char* theTaskName)
	{
	int theTaskNumber =find_task_number(theTaskName);
	if (tasks[theTaskNumber].suspended)
		{tasks[theTaskNumber].suspended = false;}
	else
		{tasks[theTaskNumber].suspended = true;}
	}

/* Function that will be run when no other functions are running. */
void idle_process();
/* Go through the task list and run any tasks that need to be run. */




/*************** HERE ARE THE SEPARATE TASKS!   *******************/

void D1_task()
    { TASK_D1_LED_TOGGLE
	  MultiFunctionShield_Single_Digit_Display (1, Task_D1_Count++);
    }

void D4_task()
    { TASK_D4_LED_TOGGLE
	  MultiFunctionShield_Single_Digit_Display (4, Task_D4_Count++);
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
     *  Example here:  Toggle the RGB_REG_TOGGLE
     *
     *  It should return before the idle period (measured in ms) has expired.
     */
    if (DIAGNOSTIC) {
        printf("Entered IDLE on tick: %ld\n\r",uwTick);
        }
     HAL_Delay(100);
    }



