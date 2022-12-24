/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h" 
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

   /*MPU6050_REGISTERS*/
#define MPU6050_ADDR 0xD0     /*Decive Addr*/
#define SMPLRT_DIV_REG 0x19   /*1khz*/
#define GYRO_CONFIG_REG 0x1B  /*Cau hinh Gryro*/
#define ACCEL_CONFIG_REG 0x1C /*Cau hinh Acc*/
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43    
#define PWR_MGMT_1_REG 0x6B    /*Wake up*/
#define WHO_AM_I_REG 0x75     /*check device*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
   /*Bluetooth Var*/
	char snum[4];
	char snum1[4];
	char str[] = { "/" };
	
   /*READ_DATA_MPU_VAR*/
int16_t Accel_X_RAW = 0 ;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Gx, Gy, Gz;
float Ax, Ay, Az;

float angle_roll_acc, angle_pitch_acc;

   /*KALMAN_FITLER*/
float Q_angle = 0.001;    
float Q_gyro = 0.003;    
float R_angle = 0.5;     
char C_0 = 1;
float dt = 0.01; // sampling time 10ms
float K1 = 0.05; 

float K_0,K_1,t_0,t_1;
float angle_err;
float gyro_bias;    // Gyroscope Drift
float angle;

float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;
	
 /*KALMAN_FITLER_2*/
float K_0_x,K_1_x,t_0_x,t_1_x;
float angle_err1;
float gyro_bias1;    // Gyroscope Drift
  
float angle1;

float Pdot_1[4] = { 0, 0, 0, 0};
float P_1[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0x, PCt_1x, Ex;


   /*PID ANGEL CONTROL*/
int PID_PWM_ANGEL; 
int PID_PWM_ANGEL1; 

float ErrorA;
float Pre_ErrorA;

float ErrorA1;
float Pre_ErrorA1;

float I_part_angel;
float I_part_angel1;

      /*Sampling time*/
#define sammpling_time_angel  0.01f; 
#define inv_sammpling_time_angel 100; 

#define sammpling_time_speed 0.04f 
#define inv_sammpling_time_speed  25; 

      /* PID Angel*/
float Set_Angel =	-3.1; //5.1

float Kp_Angel = 125  	;  //dc1: 360  
float Ki_Angel = 	0.01;
float Kd_Angel = -0.02;//0.27
	

float Set_Speed = 0; 

float Kp_Speed = 22;   //dc1: 120  //dc2: 110
float Ki_Speed = 	12;  //5
float Kd_Speed = 	0.0001;

/*********************************************************/

    /*PID Whell*/
float Set_Angel1 = -0.5;
float Kp_Angel1 = 90;
float Ki_Angel1 = 0.0001;
float Kd_Angel1 = 0.04;	

/*********************************************************/

   /*SPEED VAR*/ 
int rw = 0;
int pulseCount = 0;

int Pulse;
int rwPulse;

float speeds_filterold=0;
float speeds_filter;
float Speed;
	 
float I_part_speed;
float Pre_ErrorSP;
float ErrorSP;

int PID_PWM_SPEED;

   /*PWM OUTPUT*/
int cl;

int PWM_OUT ;	
int RE_PWM_OUT ;


int PWM_OUT1 ;
int RE_PWM_OUT1 ;

uint8_t  receive_data;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

    /*READ DATA*/ 
void MPU6050_Init (void)
  {
	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000); //check cam bien

	if (check == 0x68) //dia chi co ban cua cam bien 0x68( 110100X x là muc logic tai chan AD0)
	{
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000); // Danh thuc thiet bi

		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000); // cau hinh tan so trich mau tu cam bien ve thanh ghi (1Khz)
		
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000); //cau hinh cam bien gia toc ke.

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000); // cau hinh con quay hoi chuyen.

		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}


void MPU6050_Read_Data (void) /*note*/
{
	 
	uint8_t Rec_Data_Acc[6];
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data_Acc, 6, 1000);
	
	Accel_X_RAW = (int16_t)(Rec_Data_Acc[0] << 8 | Rec_Data_Acc [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data_Acc[2] << 8 | Rec_Data_Acc [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data_Acc[4] << 8 | Rec_Data_Acc [5]); 


	Ax = Accel_X_RAW/16384.0; /*Datasheet*/
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;			

	uint8_t Rec_Data_1_Gyro[6];	
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data_1_Gyro, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data_1_Gyro[0] << 8 | Rec_Data_1_Gyro [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data_1_Gyro[2] << 8 | Rec_Data_1_Gyro [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data_1_Gyro[4] << 8 | Rec_Data_1_Gyro [5]);
	
	Gx =  Gyro_X_RAW/131.0;
	Gy =  Gyro_Y_RAW/131.0;
	
}
      
/**/
/**/

   /*KALMAN FILTER*/      
void Kalman_Filter(double angle_m, double gyro_m)
{
		/*ESTIMATES PERIOD*/
	angle += (gyro_m - gyro_bias) * dt;   /*Cap nhat goc uoc luong ban dau*/
	
	P[0][0] += (Q_angle - P[0][1] - P[1][0]) * dt; /*Du doan buoc dau ma tran covariance, ma tran dac trung cho kha nang du doan sai so*/ 
  P[0][1] -= P[1][1] * dt;
  P[1][0] -= P[1][1] * dt;
  P[1][1] += Q_gyro * dt;
	
	  /*UPDATE KALMAN FILTER VALUE*/
  angle_err = angle_m - angle; /*Tinh sai so goc uoc luong dua vao goc cam bien vs goc uoc luong truoc*/

  E = R_angle + P[0][0];  /*Uoc tinh sai so co the xay ra*/
	
	PCt_0 = C_0 * P[0][0]; 
  PCt_1 = C_0 * P[1][0];

  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = P[0][0]; 
  t_1 = P[0][1];
  
  P[0][0] -= K_0 * t_0;  /*Cap nhat uoc tinh ma tran covariance*/ 
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  angle += K_0 * angle_err;     /*Cap nhat lai goc uoc luong sau loc.*/   
	gyro_bias += K_1 * angle_err; /*Cap nhat lai sai so gyro_bias (giua gia tri do va gia tri dung nhat). */ 
	
}
       
void Kalman_Filter_1(double angle_m, double gyro_m)
{
		/*ESTIMATES PERIOD*/
	angle1 += (gyro_m - gyro_bias1) * dt;        
		
  P_1[0][0] +=  ( Q_angle - P_1[0][1] - P_1[1][0] ) * dt;    
  P_1[0][1] -=  P_1[1][1] * dt;
  P_1[1][0] -=  P_1[1][1] * dt;
  P_1[1][1] +=  Q_gyro * dt;
  
	 /*UPDATE KALMAN FILTER VALUE*/
	angle_err1 = angle_m - angle1;
  
  Ex = R_angle + P_1[0][0];
  
	PCt_0x = C_0 * P_1[0][0];
  PCt_1x = C_0 * P_1[1][0];
	
  K_0_x = PCt_0x / Ex;
  K_1_x = PCt_1x / Ex;
  
  t_0_x = P_1[0][0];  
  t_1_x = P_1[0][1];
  
  P_1[0][0] -= K_0_x * t_0_x;    
  P_1[0][1] -= K_0_x * t_1_x;
  P_1[1][0] -= K_1_x * t_0_x;
  P_1[1][1] -= K_1_x * t_1_x;
  
	angle1 += K_0_x * angle_err1;     
  gyro_bias1 += K_1_x * angle_err1;          
}
/**/
/**/

   /*Caculate Angel*/
void Angle_Aalculate(void)
{
	 angle_pitch_acc = -atan((float)Ax / Az) * 57.296; // Y
	 angle_roll_acc = atan((float)Ay /  Az) * 57.296; //X
	
   Kalman_Filter( angle_pitch_acc, Gy); 
   Kalman_Filter_1( angle_roll_acc, Gx);     	
}      
/**/
/**/

   /*PD Control*/
void PID_Angel_Control(void)
{
	ErrorA = Set_Angel - angle; /*Ti le*/
	/*****/
	I_part_angel = I_part_angel + ErrorA * sammpling_time_angel; /*Tich phan*/
	/*****/
  PID_PWM_ANGEL = ( Kp_Angel*ErrorA )  + ( Ki_Angel*I_part_angel) + Kd_Angel*( ErrorA - Pre_ErrorA) * inv_sammpling_time_angel;
	/*****/
	Pre_ErrorA = ErrorA;
}
      
void PID_Control_wheel(void)
{
	
  ErrorA1 = Set_Angel1 - angle1;
	/*****/
	I_part_angel1 = I_part_angel1 + ErrorA1 * sammpling_time_angel;
	/*****/
  PID_PWM_ANGEL1 = ( Kp_Angel1*ErrorA1)  + ( Ki_Angel1*I_part_angel1) + Kd_Angel1*( ErrorA1 - Pre_ErrorA1) * inv_sammpling_time_angel;
	/*****/
	Pre_ErrorA1 = ErrorA1;
	
	if( PID_PWM_ANGEL1 > 999 ) PID_PWM_ANGEL1 = 999;
	else if( PID_PWM_ANGEL1 < -999 ) PID_PWM_ANGEL1 = -999;	
}	 
/**/
/**/

    /*SPEED*/
void Count_Pulse(void)  // theo chu ky PID speed (tong do lech xung cua 4 lan lay mau)
{
  rwPulse += Pulse;
  Pulse = 0;  
}
void PID_Speed_Control(void) 
{
   Speed = rwPulse * 6.8181; 
/* xung/40ms * 25 = xung/1s 
	
	xung1/s * 60 = xung/phut
	
	xung/phut chia do phan giai = vong/phut.
	
	(rwPulse * 25 * 60) / 220 = vong / phut.
	
	25*60/220 = 6.8181 => rwPulse * 6.8181 = vong / phut 
*/	
  rwPulse = 0; 
	
  speeds_filter = ( speeds_filterold * 0.7f ) + ( Speed * 0.3f ); 
  speeds_filterold = speeds_filter;

	ErrorSP = Set_Speed - speeds_filter;
	
	I_part_speed = I_part_speed + (Set_Speed - speeds_filter) * sammpling_time_speed ;
	
	PID_PWM_SPEED = ( Kp_Speed * ErrorSP ) + ( Ki_Speed * I_part_speed ) + Kd_Speed*( ErrorSP - Pre_ErrorSP )* inv_sammpling_time_speed;
	
	Pre_ErrorSP = ErrorSP;
} 

/**/
/**/

   /*PWM OUTPUT*/
void PWM_OUTPUT(void)
{
   PWM_OUT = PID_PWM_ANGEL  +  PID_PWM_SPEED;
	
	 if( PWM_OUT > 0)
		{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  if(PWM_OUT > 999) PWM_OUT = 999;
	  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, PWM_OUT );
		}
	else if(PWM_OUT < 0)
		{
		RE_PWM_OUT = -PWM_OUT;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  if (RE_PWM_OUT > 999 ) RE_PWM_OUT = 999;
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, RE_PWM_OUT);
		}
}	
    
    
void PWM_OUTPUT_wheel(void)
{
   PWM_OUT1 =  PID_PWM_ANGEL1;
	
	 if( PWM_OUT1 > 0)
		{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	  if(PWM_OUT1 > 999) PWM_OUT1 = 999;
	  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, PWM_OUT1 );
		}
	else if(PWM_OUT1 < 0)
		{
		RE_PWM_OUT1 = -PWM_OUT1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  if (RE_PWM_OUT1 > 999 ) RE_PWM_OUT1 = 999;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, RE_PWM_OUT1);
		}
}	
/**/
/**/

    /*Bluetooth*/
void Bluetooth_SPP(void)
{
    HAL_UART_Receive(&huart1,&receive_data,sizeof(receive_data),100);
	
		if( receive_data == 'A' && receive_data != 'C' )
		{
		Set_Speed = -3;
    }	
	  if( receive_data == 'B' && receive_data != 'C' )
		{
	    Set_Speed = 3; 
    }
		if( receive_data == 'C' && receive_data != 'B' && receive_data != 'A')
		{
	    Set_Speed  = 0  ;
    }
			
		sprintf(snum,"%f",Speed);
    HAL_UART_Transmit(&huart1,(uint8_t *)&snum,sizeof(Speed),100);
    HAL_UART_Transmit(&huart1,(uint8_t *)&str,sizeof(str),100);
		
		sprintf(snum1,"%f",angle);
    HAL_UART_Transmit(&huart1,(uint8_t *)&snum1,sizeof(snum1),100);
		HAL_UART_Transmit(&huart1,(uint8_t *)&str,sizeof(str),100);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
	
  HAL_TIM_Base_Start_IT(&htim10);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	
//	HAL_UART_Receive_DMA(&huart1,&receive_data,1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
   
    /* USER CODE BEGIN 3 */
	 Bluetooth_SPP();  HAL_Delay(300);
	
	 if(angle > 45 || angle < -45 || angle1 > 45 || angle1 < -45 )
	 {		 
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	 }
	 else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 19;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 19;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8400;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 99;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
	{
	    if( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==0)  Pulse++;
      if( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==1)	Pulse--;	
	}
}	
/***/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim == &htim10)
 {
   /*ANGEL*/
	MPU6050_Read_Data();
	Angle_Aalculate();
	
	/****************/
	
	PID_Angel_Control();	
	PID_Control_wheel();
	
	  /*SPEED*/
	Count_Pulse();
	 
	cl++;
	if(cl >= 4)
		{
		cl = 0;
		PID_Speed_Control();	
		}
	 /*OUTPUT*/
	PWM_OUTPUT();
	PWM_OUTPUT_wheel();
		
 }
}

/***/
/***/
/***/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
