#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include "stm32f4xx.h"
//################################## GLOBAL FUNCTION ##################################//	
	extern float baca_potensio();
	extern void InitHMC5883L();
	extern void KalibrasiHMC5883L();
	extern float baca_heading();
	extern void GetHMC5883L();
	extern void TM_EXTI_Handler(uint16_t GPIO_Pin);
	extern void CheckHeading(void);
	extern void GPIO_Initialize_Digital(void);
	extern void InitPWMOUT();
	extern void SetDriverMotor_yaw(int arah, uint8_t pwm);
	extern void SetDriverMotor_pitch(int arah, uint8_t pwm);
	extern void Pid_PITCH();
	extern void Pid_YAW();
	extern void ParsedataGPS_GNGLL(void);
	extern void ParsedataGPS_GNGGA(void);
	extern void ParsedataGPS_GNRMC(void);
	extern void GPS_Validasi(void);
	extern float dms_dd(float in_coords, char angin);
	extern void GPSAccess(void);
	extern void baca_input();
	extern void Terima_Input();
	extern void SendGCS(int waktu);
//############################################################################################//

#endif