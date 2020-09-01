//############################################ Include Library ##############################################//
	/* Include core modules */
	#include "Global_Variable.h"
	#include "stm32f4xx_gpio.h"
	#include "stm32f4xx.h"
	/* Include my libraries here */
	#include "defines.h"
	#include "string.h"
	#include "stdio.h"
	#include "stdlib.h"
	#include "math.h"
	#include "tm_stm32f4_timer_properties.h"
	#include "tm_stm32f4_pwmin.h"
	#include "tm_stm32f4_pwm.h"
	#include "tm_stm32f4_delay.h"
	#include "tm_stm32f4_usart.h"
	#include "tm_stm32f4_usart_dma.h"
	#include "tm_stm32f4_adc.h"
	#include "tm_stm32f4_exti.h"
	#include "tm_stm32f4_hmc5883l.h"
	#include "arm_math.h"

#define M_PI 3.141592654	
	/* PID */
#define KP_YAW 1.2			
#define KI_YAW 0
#define KD_YAW 50		

#define KP_PITCH 5.0
#define KI_PITCH 0
#define KD_PITCH 15
	void Parsing_Data_Wahana();
	void terima_data_wahana();
//#########DEKLARASI VARIABEL GLOBAL##########//
union Floating {
	uint8_t bytes[4];
	float value;
};
	union Doublee{
	uint8_t bytes[8];
	double value1;	
	};
//BUFFER
	 char buffer[200];
	 unsigned long timer1=0, timer2=0, timer3=0;
//VAR-CONTROL PID
	int Out_PIDPITCH;
	int Out_PIDYAW;
	float errorPitch, last_errorPitch;
	int dir_pitch = 0;
	float pitch_P,pitch_I,pitch_D;
	float KI_pitch=0;
	float yaw_P,yaw_I,yaw_D;
	int dir_yaw=0;
	float KI_yaw=0;
	float errorI_yaw;
	float errorYaw, last_errorYaw,hasilErrorYaw, pidy;
	union Floating dataHeading, dataPitch1;														//SEND GCS
//VAR-GPS
	 char gps[200];
	 char serial4[200];
	 int cek2,a2,b2,flag2,nomor_parsing2=0,ke2=0;
	 float data_lat,data_longi;
	 int data_time;
	 char lat[15],lat_char[15],longi[15],longi_char[15],valid[15],time[15];  
	 float latitude,longitude,latitude_zero,longitude_zero,selisih_gps_lat,selisih_gps_long;
	 union Floating currentLat, currentLong;
	 union Doublee currentLat1, currentLong1;
//TIM OUT
	 TM_PWM_TIM_t TIM2_Data;
	 NVIC_InitTypeDef NVIC_InitStructure;
//POTENSIO
	int  data;
	float analog;
//KOMPAS
	float headingDegrees,declinationAngle;
	float heading;
  TM_HMC5883L_t HMC5883L;
	int resetKalHMC=0;
	float magnetoX_Zero, magnetoY_Zero, magnetoZ_Zero;	
	float magnetoX, magnetoY, magnetoZ;	
	float min_magnetoX_Zero=-162.92f, min_magnetoY_Zero=-575.00f, min_magnetoZ_Zero=326.60f;
	float max_magnetoX_Zero=445.40f, max_magnetoY_Zero=163.76f, max_magnetoZ_Zero=423.20f;
//PARSE_BACA INPUT
	char serial3[100];
	char setPoint_pitchAct[10], setPoint_yawAct[10];
	int nomor_parsing5=0,flag5,a5,cek5,ke5=0,metu2,mode=0;
	float setPoint_pitch = 0, setPoint_yaw=1;
// PARSING DATA WAHANA 
	int nomor_parsing6=0,flag6,a6,cek6,ke6=0,metu3;
	char serial6[200];
	char data_modeAct[10],data_yawAct[10],data_pitchAct[10], data_rollAct[10];
	char data_windspeedAct[10], data_altitudeAct[10], data_curlatAct[20], data_curlongAct[20], data_bateraiAct[10];

	int  datamode,dataaltitude;
	union Floating datayaw, datapitch, dataroll, datawindspeed,databaterai,altitude;
	union Doublee datacurrentlat, datacurrentlong;

//######## DEKLARASI FUNGSI ########### //
//POTENSIO
float baca_potensio(){
	data = TM_ADC_Read(ADC1, ADC_Channel_15); //PC5
	analog = data;
	if (analog > 3500) analog = 3500;
	return analog*(180.0f/3500.0f);
}
//KOMPAS
void InitHMC5883L(){
	/* Init HMC5883L sensor */
	if (TM_HMC5883L_Init(&HMC5883L, TM_HMC5883L_Gain_1_3, TM_HMC5883L_OutputRate_15Hz) == TM_HMC5883L_Result_Ok) {
		/* Device OK */
		TM_USART_Puts(USART1, "\n\rHMC5883L_OK READY TO USE");
	} else {
		TM_USART_Puts(USART1, "\n\rHMC5883L_ERROR");
		/* Infinite loop */
		while (1);
	}
}
void KalibrasiHMC5883L(){
	float magX=-32768,magY=-32768, magZ=-32768;
	float minX = 32767, minY=32767, minZ=32767;
	
	while(1){
			if (TM_HMC5883L_DataReady(&HMC5883L) == TM_HMC5883L_Result_Ok)
		{
			// Read Data Kompas 
			TM_HMC5883L_Read(&HMC5883L);
			if (HMC5883L.X < minX) minX = HMC5883L.X;
			if (HMC5883L.Y < minY) minY = HMC5883L.Y;
			if (HMC5883L.Z < minZ) minZ = HMC5883L.Z;
			
			if (HMC5883L.X > magX) magX = HMC5883L.X;
			if (HMC5883L.Y > magY) magY = HMC5883L.Y;
			if (HMC5883L.Z > magZ) magZ = HMC5883L.Z;
			
			magnetoX_Zero = magnetoX_Zero + HMC5883L.X;
			magnetoY_Zero = magnetoY_Zero + HMC5883L.Y;
			magnetoZ_Zero = magnetoZ_Zero + HMC5883L.Z;
			//resetKalHMC = resetKalHMC +1;
		sprintf(buffer,"\n\r%.2f, %.2f, %.2f,|| %.2f, %.2f, %.2f", magX, magY, magZ, minX,minY,minZ);
		TM_USART_Puts(USART1, buffer);
		}
			if(resetKalHMC == 100){
				magnetoX_Zero = magnetoX_Zero/100.0;
				magnetoY_Zero = magnetoY_Zero/100.0;
				magnetoZ_Zero = magnetoZ_Zero/100.0;
				TM_USART_Puts(USART1,"\n\rALHAMDULILLAH KALIBRASI KOMPAS BERHASIL...");
				Delayms(1000);
				resetKalHMC=0;
				break;
			}
		}	
}
float baca_heading(){
		// Calculate the angle of the vector x,y
		/// 3.141592654;
		heading = (atan2(magnetoY,magnetoX)); 
	
		declinationAngle = 0.22;
		heading += declinationAngle;
		
		// Normalized to 0 - 360
		if (heading < 0)
		{
			heading += 2*PI;
		}
		if (heading > 2*PI)
		{
			heading -= 2*PI;
		}
		// Conv Rad To Degree
		 headingDegrees = heading * 180/M_PI;
		
		if ( headingDegrees < 0) headingDegrees+=360;
		else if(headingDegrees > 360) headingDegrees-=360;
		
		return headingDegrees;
}
void GetHMC5883L(){
	if (TM_DELAY_Time() -timer3>= 50){
		if (TM_HMC5883L_DataReady(&HMC5883L) == TM_HMC5883L_Result_Ok)
		{
		// Read Data Kompas 
			TM_HMC5883L_Read(&HMC5883L);
			magnetoX = HMC5883L.X - ((min_magnetoX_Zero+max_magnetoX_Zero)/2);
			magnetoY = HMC5883L.Y - ((min_magnetoY_Zero+max_magnetoY_Zero)/2);
			magnetoZ = HMC5883L.Z - ((min_magnetoZ_Zero+max_magnetoZ_Zero)/2);
			dataHeading.value = baca_heading();
		}
	} 
}
void TM_EXTI_Handler(uint16_t GPIO_Pin) { /* Main EXTI handler Kompas */
	/* If data ready interrupt */
	if (GPIO_Pin == HMC5883L_DRDY_PIN) {
		/* Process interrupt */
		TM_HMC5883L_ProcessInterrupt(&HMC5883L);
	}
}
void CheckHeading(void){
		  if (dataHeading.value == 0.0) {
					TM_USART_Puts(USART1, "\n\rINVALID"); return;
			}
}
//++++++++++++++++++++++++++++++++++++++++++DRIVER MOTOR L293D++++++++++++++++++++++++++++++++++++++++//											
void GPIO_Initialize_Digital(void){
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_5;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
}
void InitPWMOUT(){ //PWM OUT
	 /* Set PWM to 1kHz frequency on timer TIM2 */
    /* 1 kHz = 1ms = 1000us */
  TM_PWM_InitTimer(TIM2, &TIM2_Data, 1000);
	//Driver Motor
	TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_2, TM_PWM_PinsPack_2);		  // Pb3
	TM_PWM_InitChannel(&TIM2_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_3);			// pa15
	}
									 //Driver Motor
void SetDriverMotor_yaw(int arah, uint8_t pwm){
		switch(arah) {
		case 0:
			GPIO_SetBits(GPIOA, GPIO_Pin_5);			//PA5_HIGH
			GPIO_ResetBits(GPIOA, GPIO_Pin_3);			//PA3_LOW
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, pwm*2);
			break;
		case 1:
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);	 //PA5_LOW
			GPIO_SetBits(GPIOA, GPIO_Pin_3);			//PA3_HIGH
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_1, pwm*2);  // Set channel 1 value, 500us pulse high = 500 / 1000 = 0.5 = 50% duty cycle 
			break;
	}
}
void SetDriverMotor_pitch(int arah, uint8_t pwm){
	switch(arah) {
		case 0:
			GPIO_SetBits(GPIOA, GPIO_Pin_2);			//PA2_HIGH
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);		//PA1 LOW
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_2, pwm*7);
			break;
		case 1:
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);			//PA2_LOW
			GPIO_SetBits(GPIOA, GPIO_Pin_1);				//PA1 HIGH
			TM_PWM_SetChannelMicros(&TIM2_Data, TM_PWM_Channel_2, pwm*4);
			break;
	} 
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//PID CONTROL
void Pid_PITCH(){
		errorPitch = setPoint_pitch - dataPitch1.value;
		int maxPitchSpeed = 200;
	if (errorPitch < 0 ){
		dir_pitch = 0; //TURUN
			maxPitchSpeed = 90;
	} else {
		dir_pitch = 1; //NAIK
	}

	pitch_P  = KP_PITCH *errorPitch; 
	pitch_I += KI_PITCH * errorPitch; 
	pitch_D  = KD_PITCH *(errorPitch - last_errorPitch) ;
	
	if (-5.0f < errorPitch && errorPitch < 5.0f){
		if(pitch_I > 90) pitch_I = 90;
		else if(pitch_I < -90) pitch_I = -90;
		KI_pitch = 0.5;
	}
	else {
		KI_pitch = 0.9;
	}
	Out_PIDPITCH = pitch_P + pitch_I + pitch_D;
	
	if(Out_PIDPITCH < 0 ) Out_PIDPITCH *= -1;
	if(Out_PIDPITCH > maxPitchSpeed) Out_PIDPITCH = maxPitchSpeed;
	
	if(errorPitch == 0) Out_PIDPITCH = 0;
	
	SetDriverMotor_pitch(dir_pitch,  Out_PIDPITCH );

  last_errorPitch = errorPitch;
}
void Pid_YAW(){
	errorYaw = setPoint_yaw - dataHeading.value;
		
	if(dataHeading.value >= 265 && setPoint_yaw <= 95){
		errorYaw += 359;
	}else if(dataHeading.value <= 95 && setPoint_yaw >= 265) {
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
		if(yaw_I > 120) yaw_I = 120;
		else if(yaw_I < -120) yaw_I = -120;
		if(-2.0f < errorYaw && errorYaw < 2.0f) KI_yaw = 0.2;
		else(KI_yaw = 0.4);
	}
	else {
		KI_yaw = 0.4;
	}
	Out_PIDYAW = yaw_P + yaw_I + yaw_D;
	//Out_PIDYAW = yaw_P + yaw_D;
	
	if (Out_PIDYAW < 0) Out_PIDYAW *= -1;
	if (Out_PIDYAW > 250) Out_PIDYAW = 250;

	if(-1.5f < errorYaw && errorYaw < 1.5f) Out_PIDYAW =0;
	SetDriverMotor_yaw(dir_yaw,Out_PIDYAW);
	last_errorYaw = errorYaw; 
} 

// GPS LAT LONG
void ParsedataGPS_GNGLL(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  lat_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 4)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void ParsedataGPS_GNGGA(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  lat_char[ke2] = gps[cek2]; 
									}
									else if (nomor_parsing2 == 4)
									{
										longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void ParsedataGPS_GNRMC(void){
				if (flag2==2)
				{
					for(cek2=a2; cek2<sizeof(gps);cek2++)
					{
							if(gps[cek2] == ',')
								{
										nomor_parsing2++;
										ke2 = 0;
										continue;
								}
							else
								{				
									if (nomor_parsing2 == 1)
									{
										time[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 2)
									{
									  valid[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 3)
									{
									  lat[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 4)
									{
										lat_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 5)
									{
										longi[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 == 6)
									{
										longi_char[ke2] = gps[cek2];
									}
									else if (nomor_parsing2 > 6)
									{
										nomor_parsing2 = 0;
										flag2 = 0;
										break;
									}
									ke2++;	
								}
						}
				}
			data_time	=	atoi(time);
			data_lat = atof(lat);
			data_longi = atof(longi);
}
void GPS_Validasi(void){
				for (cek2 = 0; cek2 < sizeof(gps); cek2++){
					if(('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('G' == gps[cek2+3]) && ('L' == gps[cek2+4]) && ('L' == gps[cek2+5]) && (flag2 == 0))
					{
						if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
						}
						ParsedataGPS_GNGLL();
					}
					else if (('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('G' == gps[cek2+3]) && ('G' == gps[cek2+4]) && ('A' == gps[cek2+5]) && (flag2 == 0))
					{
						if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
						}
						ParsedataGPS_GNGGA();
					}
					else if(('$' == gps[cek2]) && ('G' == gps[cek2+1]) && ('N' == gps[cek2+2]) && ('R' == gps[cek2+3]) && ('M' == gps[cek2+4]) && ('C' == gps[cek2+5]) && (flag2 == 0))
					{
							if(flag2 == 0) {
							a2 = cek2+6;
							flag2 = 2;
					}
							ParsedataGPS_GNRMC();
				}
			}
}
float dms_dd(float in_coords, char angin){
	float f=in_coords;
	char arah=angin;
//	float per;
	int firstdig=((int)f)/100;
	float nextdig=f-(float)(firstdig*100);
		
	if('W'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*-1.0);
		return final;
	}
		
	if('N'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*1.0);
		return final;
	}
	
	if('E'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*1.0);
		return final;
	}
	
	if('S'==arah) {
		float final=(float)((firstdig+(nextdig/60.0))*-1.0);
		return final;	
	}
// W-,N+,E+,S-
	}

void GPSAccess(void){
			if (TM_USART_Gets(USART3, serial4, sizeof(serial4))) {		// GPS 
			strcpy(gps, serial4);
			GPS_Validasi();
			currentLat.value  	= dms_dd(data_lat,lat_char[0]);
			currentLong.value 	= dms_dd(data_longi,longi_char[0]);
			currentLat1.value1 	=  currentLat.value;
			currentLong1.value1 =  currentLong.value;
		}
}
//ParseData_BacaInput
void baca_input(){
		for(cek5=0; cek5<sizeof(serial3); cek5++)
			{
					if('i' == serial3[cek5] && flag5==0)
						{
							if(flag5==0){
								a5=cek5+1;
								flag5=2;
							}
								break;
						}
			}
				if (flag5==2)
				{
					for(cek5=a5; cek5<sizeof(serial3);cek5++)
					{
							if(serial3[cek5] == ',')
								{
										nomor_parsing5++;
										ke5 = 0;
										continue;
								}
							else{
									if (nomor_parsing5 == 1){
										setPoint_pitchAct[ke5] = serial3[cek5]; 
									}
									else if (nomor_parsing5 == 2){
										setPoint_yawAct[ke5] = serial3[cek5]; 
									}
									else if (nomor_parsing5 > 2){
										nomor_parsing5 = 0;
										flag5 = 0;
										metu2=1;
										break;
									}
									ke5++;	
								}
						}
				}
			setPoint_pitch = atof(setPoint_pitchAct);
			setPoint_yaw   = atof(setPoint_yawAct);
			if (setPoint_pitch > 90) setPoint_pitch = 90;
			if (setPoint_pitch <  0) setPoint_pitch = 0;
			if (setPoint_yaw > 359) setPoint_yaw= 359;
			if (setPoint_yaw <  1 ) setPoint_yaw = 1;			
	}
void Terima_Input(){
		if (TM_USART_Gets(USART1, serial3, sizeof(serial3))) baca_input();			//PARSING DATA DARI GCS
}

void Parsing_Data_Wahana(){
	for(cek6=0; cek6<sizeof(serial6); cek6++)
			{
					if(('t' == serial6[cek6])&& (flag6==0))
						{
							if(flag6==0){
								a6=cek6+1;
								flag6=9;
							}
								break;
						}
			}
				if (flag6==9)
				{
					for(cek6=a6; cek6<sizeof(serial6);cek6++)
					{
							if(serial6[cek6] == ',')
								{
										nomor_parsing6++;
										ke6 = 0;
										continue;
								}
							else{
									if (nomor_parsing6 == 1){
										data_modeAct[ke6] = serial6[cek6]; 
									}
									else if (nomor_parsing6 == 2){
										data_yawAct[ke6] = serial6[cek6]; 
									}
									else if (nomor_parsing6 == 3) {
										data_pitchAct[ke6] = serial6[cek6];
									}
									else if (nomor_parsing6 == 4) {
										data_rollAct[ke6] = serial6[cek6];
									}
									else if (nomor_parsing6 == 5) {
										data_windspeedAct[ke6] = serial6[cek6];
									}
									else if (nomor_parsing6 == 6) {
										data_altitudeAct[ke6] = serial6[cek6];
									}
									else if (nomor_parsing6 == 7) {
										data_curlatAct[ke6] = serial6[cek6];
									}
									else if (nomor_parsing6 == 8) {
										data_curlongAct[ke6] = serial6[cek6];
									}	
									else if (nomor_parsing6 == 9) {
										data_bateraiAct[ke6] = serial6[cek6];
									}									
									else if (nomor_parsing6 > 9){
										nomor_parsing6 = 0;
										flag6 = 0;
										metu3=1;
										break;
									}
									ke6++;	
								}
						}
				}
			datamode 							 = atoi(data_modeAct);
			datayaw.value					 = atof(data_yawAct);
			datapitch.value				 = atof(data_pitchAct);
			dataroll.value				 = atof(data_rollAct);
			datawindspeed.value		 = atof(data_windspeedAct);
			dataaltitude			 		 = atoi(data_altitudeAct);
			datacurrentlat.value1	 = atof(data_curlatAct);
			datacurrentlong.value1 = atof(data_curlongAct);
			databaterai.value			 = atof(data_bateraiAct);	
			altitude.value = dataaltitude;
}
void terima_data_wahana(){
			if (TM_USART_Gets(USART3, serial6, sizeof(serial6))) 	Parsing_Data_Wahana(); 
}
// Send GCS
void SendGCS(int waktu){
		if (TM_DELAY_Time() -timer1 >= waktu){	
				// ######################### Aktifkan untuk debug data #######################################//
							sprintf(buffer,"%d,%.2f,%.2f,%.2f,%.2f,%d,%f,%f,%.2f,%.2f,%.2f\n\r",
									datamode,datayaw.value,datapitch.value,dataroll.value,datawindspeed.value,
								dataaltitude,datacurrentlat.value1,datacurrentlong.value1,databaterai.value, dataPitch1.value, dataHeading.value);
			
	/*		
			//FORMAT DATA GCS ENKRIP
			  TM_USART_Putc(USART1, datamode);
				TM_USART_Send(USART1, datayaw.bytes    						,4);
				TM_USART_Send(USART1, datapitch.bytes  						,4);
				TM_USART_Send(USART1, dataroll.bytes   						,4);
				TM_USART_Send(USART1, datawindspeed.bytes  				,2);
				TM_USART_Send(USART1, altitude.bytes							,4);
				TM_USART_Send(USART1, datacurrentlat.bytes  			,8);
				TM_USART_Send(USART1, datacurrentlong.bytes   		,8);
				TM_USART_Send(USART1, databaterai.bytes						,4);
				TM_USART_Send(USART1, dataPitch1.bytes 						,4);
				TM_USART_Send(USART1, dataHeading.bytes						,4);				
				TM_USART_Putc(USART1,0x0A);
			*/	
			TM_USART_Send(USART1, (uint8_t *)buffer, strlen(buffer));
			timer1 = TM_DELAY_Time();
		}
}
//#####################################################################################################################################################################################################################################################//

// MAIN PROGRAM 
int main(void){
	SystemInit(); 		// Initialize system //
	TM_DELAY_Init(); 	// Initialize delay //
	
	// Initialize USART1, 115200 baud, TX: PA9 RX:PA10     		    
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 57600); 				//WIFI
	// Initialize USART3    9600 baud, TX: PC10, RX: PC11 
	TM_USART_Init(USART3, TM_USART_PinsPack_3, 57600);			  //TELEMETRY  
	
	GPIO_Initialize_Digital(); 	// Initialize Digital Pin
	InitPWMOUT();							 // Initialize PWM OUT
	TM_ADC_Init(ADC1, ADC_Channel_15); //Initialize Adc_Channel_15 PIN:PC5
	
	InitHMC5883L();		// Initialize Sensor Kompas
	
	//==========Aktifkan Bila Kalibrasi===========//
							//KalibrasiHMC5883L(); 
  //============================================//
//########>>>>>>>>START<<<<<<<########//
	TM_USART_Puts(USART1,"\n\rBISMILLAH SYSTEM EFRISA TRACKER MEMULAI... ");
	Delayms(2000);
	while(1){
			// =================CEK MOTOR DRIVER====================//
			//SetDriverMotor_yaw(1,255);		//HIGH LOGIC PUTAR KIRI
			//SetDriverMotor_pitch(1,250);  //LOW  LOGIC PUTAR KANAN
			//======================================================//
			terima_data_wahana();								// data_wahana
			GPSAccess();			  								// Aktif GPS
			GetHMC5883L();				             //  Baca Kompas sudah include data heading
			dataPitch1.value = baca_potensio();			 //  Baca Potensio
			CheckHeading();                    //  Check Error Data Heading
			Terima_Input();										 //  PARSING Input SetPoint_SUDUT PITCH YAW WAHAN
			//MOTOR DRIVER YAW PITT
			if (TM_DELAY_Time() -timer2 >= 50){	
					Pid_PITCH();
					Pid_YAW();
				timer2 = TM_DELAY_Time();
			}
		//	SetDriverMotor_yaw(dir_yaw,Out_PIDYAW);
		//	SetDriverMotor_pitch(dir_pitch,Out_PIDPITCH);
// #################### RUN FLIGHT MODE ################### //
//Kirim Ke Ground Control Station
		SendGCS(100);   
	}
}