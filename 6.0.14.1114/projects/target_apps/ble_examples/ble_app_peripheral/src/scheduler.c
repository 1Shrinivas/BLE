///*Scheduler Functionality implementaion FILE */
///* The Scheduler will run every one hour duration */
///* HR,SPO2,CBT parameters are run in the scheduler */

//#include "User_Application.h"

//#define HOUR_CONVERSION       (60 * 100)
//#define FIVE_MINUTES					(300 * 100)
//#define ONE_HOUR      				(60) * HOUR_CONVERSION

//#define DEFAULT_SCH_TIME			 60
//#define MIN_DELAY_ALLOWED		 	 30
//#define MAX_DELAY_ALLOWED      690 // Delay should not be more than maximum allowed
//#define SOS_INTERVAL					 FIVE_MINUTES
///*******************************************************************************************/
//uint8_t Sch_Timer_Id    __SECTION_ZERO("retention_mem_area0");
//uint32_t Scheduler_Time_Delay __SECTION_ZERO("retention_mem_area0");
///*******************************************************************************************/

//# define WAIT_TIME												 0x32
//# define HR_MEASURING_MAX_TIME   					 WAIT_TIME * 100                  // sec
//# define SPO2_MEASURING_MAX_TIME   				 WAIT_TIME * 100
//# define TEMPERATURE_MEASURING_MAX_TIME    WAIT_TIME * 100
//# define SCHEDULER_TIME_DEBUG								180 * 100 // 6 minutes

//uint8_t scheduler_running = 0;

//static void SchedulerCallback()
//{
//	uint8_t ret = 0xff;
////	ret = notify_stm(INDICATION, comm_wScheduler, NULL, 0);
//	if(ret == 0)
//	{
//		scheduler_running = true; //TODO:need to enable
//	}	
//	Sch_Timer_Id = app_easy_timer(Scheduler_Time_Delay, SchedulerCallback);
// 
//}

//void InitialiseScheduler(void)
//{
//	if(Sch_Timer_Id == EASY_TIMER_INVALID_TIMER)
//	{
//		Scheduler_Time_Delay 		= DEFAULT_SCH_TIME * HOUR_CONVERSION;
//		Sch_Timer_Id = app_easy_timer(Scheduler_Time_Delay, SchedulerCallback) ;
//	}
//}

//void ModifySchTime(uint32_t Delay)
//{
//	
//		if( (Delay < MIN_DELAY_ALLOWED) || (Delay > MAX_DELAY_ALLOWED))
//		return ; 		

//    Scheduler_Time_Delay = Delay * HOUR_CONVERSION ;

//    if(Sch_Timer_Id)
//    app_easy_timer_modify(Sch_Timer_Id , Scheduler_Time_Delay) ;

//}


//void Stop_Scheduler(void)
//{		
//		if( Sch_Timer_Id )
//		app_easy_timer_cancel( Sch_Timer_Id ); // Cancel Scheduler 
//		Sch_Timer_Id = EASY_TIMER_INVALID_TIMER; // Reset the Timer ID
//}
