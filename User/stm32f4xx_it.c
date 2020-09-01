/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "tm_stm32f4_delay.h"
#include "main.h"
#include "Global_Variable.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
	/*
	extern float KP_YAW,KI_YAW,KD_YAW;				
	extern float KP_PITCH,KI_PITCH, KD_PITCH;
	extern float setPoint_yaw, setPoint_pitch;
	extern float dataHeading;
	extern int Out_PIDPITCH;
	extern int Out_PIDYAW;
	extern float errorPitch, last_errorPitch, dataPitch;;
	extern int dir_pitch;
	extern float pitch_P,pitch_I,pitch_D;
	extern float KI_pitch;
	extern float yaw_P,yaw_I,yaw_D;
	extern int dir_yaw;
	extern float KI_yaw;
	extern float errorI_yaw;
	extern float errorYaw, last_errorYaw, dataHeading, hasilErrorYaw, pidy;
	*/
/******************************************************************************/
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function decrement timing variable
  *	@with __weak parameter to prevent errors
  * @param  None
  * @retval None
  */
__weak void TimingDelay_Decrement(void) {

}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//PID CONTROL TRACKER 
/*
void Pid_YAW(){
	errorYaw = setPoint_yaw - dataHeading;
		
	if(dataHeading >= 265 && setPoint_yaw <= 95){
		errorYaw += 359;
	}else if(dataHeading <= 95 && setPoint_yaw >= 265) {
		errorYaw += 359;
		errorYaw *= -1;
	}
	if(errorYaw < 0){
		dir_yaw = 1;
	}else {
		dir_yaw = 0;
	}

	yaw_P = KP_YAW *errorYaw;
	yaw_I += KI_yaw * errorYaw;
	yaw_D = KD_YAW *(errorYaw - last_errorYaw);
	
	if(-5.0f < errorYaw && errorYaw < 5.0f){
		if(yaw_I > 20) yaw_I = 20;
		else if(yaw_I < -20) yaw_I = -20;
		if(-2.0f < errorYaw && errorYaw < 2.0f) KI_yaw = 0.01;
		else(KI_yaw = 0.02);
	}
	else {
		KI_yaw = 0.04;
	}
	Out_PIDYAW = yaw_P + yaw_I + yaw_D/0.001;;
	//Out_PIDYAW = yaw_P + yaw_D;
	
	if (Out_PIDYAW < 0) Out_PIDYAW *= -1;
	if (Out_PIDYAW > 250) Out_PIDYAW = 250;

	if(-1.5f < errorYaw && errorYaw < 1.5f) Out_PIDYAW =0;
	last_errorYaw = errorYaw; 
}
void Pid_PITCH(){
		uint8_t maxPitchSpeed = 200;
	if (errorPitch < 0 ){
		dir_pitch = 0; //TURUN
			maxPitchSpeed = 150;
	} else {
		dir_pitch = 1; //NAIK
	}
	errorPitch = setPoint_pitch - dataPitch;
	
	pitch_P  = KP_PITCH *errorPitch; 
	pitch_I += KI_PITCH * errorPitch; 
	pitch_D  = KD_PITCH *(errorPitch - last_errorPitch) ;
	
	if (-8.0f < errorPitch && errorPitch < 8.0f){
		if(pitch_I > 65) pitch_I = 65;
		else if(pitch_I < -65) pitch_I = -65;
		KI_pitch = 0.1;
	}
	else {
		KI_pitch = 0.2;
	}
	Out_PIDPITCH = pitch_P + pitch_I + pitch_D/0.001;;
	
	if(Out_PIDPITCH < 0 ) Out_PIDPITCH *= -1;
	if(Out_PIDPITCH > maxPitchSpeed) Out_PIDPITCH = maxPitchSpeed;
	
	if(errorPitch == 0) Out_PIDPITCH = 0;
  last_errorPitch = errorPitch;
}
*/
void SysTick_Handler(void)
{
	//Pid_YAW();
	//Pid_PITCH();
	Delayms(1);
	TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
