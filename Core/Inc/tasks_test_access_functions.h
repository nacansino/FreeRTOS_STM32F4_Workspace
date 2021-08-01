/**
 * @file tasks_test_access_functions.h
 * @author Niel Cansino (nielcansino@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef TASKS_TEST_ACCESS_FUNCTIONS_H_
#define TASKS_TEST_ACCESS_FUNCTIONS_H_

/**
 * @brief Gets the size of the TCB
 * 
 * @return size_t Size of the TCB in bytes
 */
size_t uxTaskGetTCBSize(void)
{
	return sizeof(TCB_t);
}

/**
 * @brief Gets the total stack size of a particular task
 * 
 * @param xTask     Task Handle
 * @return uint32_t Size of the task stack in bytes
 */
uint32_t uxTaskGetStackSize( TaskHandle_t xTask )
{
   uint32_t uxReturn;
   TCB_t *pxTCB;

   if( xTask != NULL )
   {
       pxTCB = ( TCB_t * ) xTask;
       uxReturn = (pxTCB->pxEndOfStack - pxTCB->pxStack) * sizeof(StackType_t);
   }
   else
   {
       uxReturn = 0;
   }

   return uxReturn;
}

#endif
