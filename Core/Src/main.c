/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

#include "math.h"
#include "stdbool.h"

#include "stdlib.h"
#include "ibus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _USE_MATH_DEFINES
#define UART3_DMA_BUF_SIZE 128
uint8_t uart3_dma_buf[UART3_DMA_BUF_SIZE];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char* p, int len){
	for(int i=0; i<len; i++){
		ITM_SendChar((*p++));
	}
	return len;
}
// EBIMU set function
#define DMA_BUF_SIZE 128
#define SBUF_SIZE 64

uint8_t uart1_dma_buf[DMA_BUF_SIZE];
volatile uint16_t dma_head = 0;

char sbuf[SBUF_SIZE];
int sbuf_cnt = 0;

static float roll = 0, pitch = 0, yaw = 0;

// === IDLE interrupt Handler ===
void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)

    {
        uint16_t dma_tail = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

        while (dma_head != dma_tail) {
            char c = uart1_dma_buf[dma_head++];
            if (dma_head >= DMA_BUF_SIZE) dma_head = 0;

            if (c == '*') {
                sbuf_cnt = 0;
                memset(sbuf, 0, SBUF_SIZE);
            }

            if (sbuf_cnt < SBUF_SIZE - 1) {
                sbuf[sbuf_cnt++] = c;
            }

            if (c == '\n') {
                sbuf[sbuf_cnt] = '\0';
                float r, p, y;
                if (sscanf(sbuf, "*%f,%f,%f", &r, &p, &y) == 3) {
                    roll = r;
                    pitch = p;
                    yaw = y;
                }
                sbuf_cnt = 0;
            }
        }
    }
}



void ODrive_Send_CAN_Message(uint32_t id, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = id;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    for (int attempt = 0; attempt < 3; ++attempt) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) == HAL_OK) {
                return; // Success
            }
        }
        HAL_Delay(1); // Retry after short delay
    }

    printf("CAN TX failed: id=0x%X\r\n", id);
}

// ODrive velocity set ----------------------------------------------
void ODrive_Set_Input_Velocity(uint8_t node_id, float velocity) {
    uint32_t cmd_id =  ((uint32_t)node_id << 5) | (0x0d);
    uint8_t data[8];
    float torque_ff = 0.0f;
    memcpy(data, &velocity, 4);      // velocity
    memcpy(data + 4, &torque_ff, 4); // torque feed-forward
    ODrive_Send_CAN_Message(cmd_id, data, 8);
}//velocity Unit = rev/s

// ODrive Axis set ----------------------------------------------
static uint32_t axis_state_1 = 0;
static uint32_t axis_state_2 = 0;
void ODrive_Set_Axis_State(uint8_t node_id, uint32_t state) {
    uint32_t cmd_id = ((uint32_t)node_id << 5) | (0x07);
    uint8_t data[4];
    memcpy(data, &state, 4);
    ODrive_Send_CAN_Message(cmd_id, data, 4);
}
// ODrive reboot ----------------------------------------------
void ODrive_reboot(uint8_t node_id){
	uint32_t cmd_id =  ((uint32_t)node_id << 5) | (0x16);
	uint8_t data[8] = {0};
	ODrive_Send_CAN_Message(cmd_id, data, 8);
}
// ODrive Axis state ----------------------------------------------
void ODrive_Get_Axis_State(uint8_t node_id) {
    uint32_t cmd_id = ((uint32_t)node_id << 5) | (0x01);
    uint8_t data[0];
    ODrive_Send_CAN_Message(cmd_id, data, 0);
}


// ODrive Torque set ----------------------------------------------
void ODrive_Set_Input_Torque(uint8_t node_id, float Torque) {
	uint32_t cmd_id =  ((uint32_t)node_id << 5) | (0x0e);
	uint8_t data[4];
    memcpy(data, &Torque, 4); // torque
    ODrive_Send_CAN_Message(cmd_id, data, 4);
}

//ODrive encoder data request ----------------------------------------------
void ODrive_Get_Encoder_Estimates(uint8_t node_id) {
	uint32_t cmd_id =  ((uint32_t)node_id << 5) | (0x09);
	uint8_t data[0];
	ODrive_Send_CAN_Message(cmd_id, data, 0);
}
static float en_pos_w = 0.0f,  en_vel_w = 0.0f; // Initial wheel motor pos, vel
static float en_pos_r = 0.0f, en_vel_r = 0.0f; // Initial reaction wheel motor pos, vel

//ODrive encoder data receive ----------------------------------------------
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        // CAN ID = 0x29,  wheel motor encoder position and velocity
        if (rxHeader.StdId == 0x29) {
            memcpy(&en_pos_w, &rxData[0], 4);
            memcpy(&en_vel_w, &rxData[4], 4);
        }

        // CAN ID = 0X49, reaction wheel motor encoder position and velocity
        else if (rxHeader.StdId == 0x49) {  // 0x009 + (2 << 5)
            memcpy(&en_pos_r, &rxData[0], 4);
            memcpy(&en_vel_r, &rxData[4], 4);
        }
        else if (rxHeader.StdId == 0x01 + (1 << 5)) { // CAN 1 axis check
            memcpy(&axis_state_1, rxData, 4);
        }
        else if (rxHeader.StdId == 0x01 + (2 << 5)) { // CAN 1 axis check
			memcpy(&axis_state_2, rxData, 4);
		}

    }
}



// Initial value set ----------------------------------------------
float targetAngle_w = - 1.0f;
float O_targetAngle_w = - 1.0f;
float iE_w;
float prevE_w;
float prevY_w;
static float tq_w;

// Wheel motor Constants ----------------------------------------------
//static float Kp_w = 0.4, Ki_w = 0.05, Kd_w = 0.015; //PID gain
//static float Kp_w = 0.137, Ki_w = 0.0015, Kd_w = 0.000082; //PID gain
//static float Kp_w = 0.2, Ki_w = 1.5, Kd_w = 0.0019; //PID gain
float Kp_w = 0.3, Ki_w = 10.0, Kd_w = 0.0016; //PID gain
//static float Kp_w = 0.25, Ki_w = 10.0, Kd_w = 0; //PID gain
//static float AF1_w = 0.00002, AF2_w = 0.00003; // Angle Fixrate
static float AF1_w = 0.00004, AF2_w = 0.001; // Angle Fixrate
float O_AF1_w = 0.00004, O_AF2_w = 0.001;
static float dt_w = 0.0002; //Time period
static float AD_w = 1.0; //Adjusting constants
float PIDoutput_w;
float tq_lim_w = 3;
float iE_lim_w = 5;
static float error_w_f;
float dY_w;
float dY_w_f;
float alpha_w = 0.999; //LPF constant
float beta_w = 0.9;

int e,ec;


// Initial value set ----------------------------------------------
float targetAngle_r = 0.0f;
float O_targetAngle_r = 0.0f;
float iE_r;
float prevE_r;
float prevY_r;
static float tq_r;

// reaction Wheel motor Constants ----------------------------------------------
float Kp_r = 0.18, Ki_r = 0.2, Kd_r = 0.0005; //PID gain
//static float AF1_r = 0.0001, AF2_r = 0.000001; // Angle Fixrate
//static float AF1_r = 0.000035, AF2_r = 0.0005; // Angle Fixrate
//static float AF1_r = 0.0001, AF2_r = 0.000012; // Angle Fixrate
static float AF1_r = 0.00025, AF2_r = 0.000015; // Angle Fixrate
float O_AF1_r = 0.00025, O_AF2_r = 0.000015;
static float dt_r = 0.0004; //Time period
static float AD_r = 1.0; //Adjusting constants
float PIDoutput_r;
float tq_lim_r = 5.0;
float iE_lim_r = 7.0;

static float error_r_f;
float dY_r;
float dY_r_f;
float alpha_r = 0.99; //LPF constant
float beta_r = 0.9;


/* ---------------------------------------------- BALANCING ---------------------------------------------- */
bool balance_enabled_w = true;// Fail check bool
bool balance_enabled_r = true;// Fail check bool


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Wheel motor Balancing ----------------------------------------------
    if (htim->Instance == TIM1) {

    	// Fail mode ----------------------------------------------
        if (fabs(pitch) > 20 || fabs(roll) > 15) {
            if (balance_enabled_w) {
            	ODrive_Set_Input_Torque(1, 0.0f);// Wheel motor stop
			 	tq_w = 0;
			 	iE_w = 0;
			 	prevE_w = 0;
                balance_enabled_w = false;
            }
            return;
        }

        if (!balance_enabled_w && fabs(pitch) < 15 && fabs(roll) < 10) {
            balance_enabled_w = true;// if pitch and roll return 15 deg, PID revive
        }

        if (!balance_enabled_w) return;

    	//PID Output ----------------------------------------------
        	float error_w = targetAngle_w - pitch;
        	error_w_f = alpha_w * error_w + (1.0 - alpha_w) * error_w_f;
        	iE_w += error_w_f * dt_w;
        	if (iE_w > iE_lim_w) {
        		iE_w = iE_lim_w;
			}
			else if (iE_w < - iE_lim_w) {
				iE_w = - iE_lim_w;
			}
        	float dE_w = (error_w_f - prevE_w) / dt_w;
        	prevE_w = error_w_f;

        	PIDoutput_w = Kp_w * error_w_f + Ki_w * iE_w + Kd_w * dE_w;
        	//Kp_w = Kp_w_old; Kd_w = Kd_w_old;

        	//I-PD Output ----------------------------------------------
        /*
        	float error_w = targetAngle_w - pitch;
        	error_w_f = alpha_w * error_w + (1.0 - alpha_w) * error_w_f;
        	iE_w += error_w_f * dt_w;  // Integral term on error
        	dY_w = (pitch - prevY_w) / dt_w;  // Derivative on output
        	dY_w_f = beta_w * dY_w + (1.0 - beta_w) * dY_w_f;
        	PIDoutput_w = Ki_w * iE_w - Kp_w * pitch - Kd_w * dY_w_f;
        	prevY_w = pitch;
        */

        	//Motor torque ----------------------------------------------
        	tq_w = AD_w * PIDoutput_w;

        	// Clamp torque
        	if (tq_w > tq_lim_w) {
        		tq_w = tq_lim_w;
        		iE_w -= error_w_f * dt_w;  // anti-windup
        	}
        	else if (tq_w < -tq_lim_w) {
        		tq_w = -tq_lim_w;
        		iE_w -= error_w_f * dt_w;  // anti-windup
        	}
        	ODrive_Set_Input_Torque(1, (float)tq_w);
        	//encoder value
        	ODrive_Get_Encoder_Estimates(1);
        	//Target Angle ----------------------------------------------
        	if (targetAngle_w > pitch) {
        		targetAngle_w += 1.0 * AF1_w;
        	}
        	if (targetAngle_w <= pitch) {
        		targetAngle_w -= 1.0 * AF1_w;
        		}
        	targetAngle_w += AF2_w * en_vel_w;


          }

    // Reaction Wheel motor Balancing ----------------------------------------------
    if (htim->Instance == TIM2) {
    	// Fail mode ----------------------------------------------
    	if (fabs(pitch) > 20 || fabs(roll) > 15) {
			if (balance_enabled_r) {
				ODrive_Set_Input_Torque(2, 0.0f);// Reaction wheel motor stop
				tq_r = 0;
				iE_r = 0;
				prevE_r = 0;
				balance_enabled_r = false;
			}
			return;
		}

		if (!balance_enabled_w && fabs(pitch) < 15 && fabs(roll) < 10) {
			balance_enabled_r = true;// if pitch and roll return 15 deg, PID revive
		}
		if (!balance_enabled_r) return;
		//PID Output ----------------------------------------------
		float error_r = targetAngle_r - roll;
		error_r_f = alpha_r * error_r + (1.0 - alpha_r) * error_r_f;
		iE_r += error_r_f * dt_r;
		if (iE_r > iE_lim_r) {
    		iE_r = iE_lim_r;
		}
		else if (iE_r < - iE_lim_r) {
			iE_r = - iE_lim_r;
		}
		float dE_r = (error_r_f - prevE_r) / dt_r;
		prevE_r = error_r_f;

		PIDoutput_r = Kp_r * error_r_f + Ki_r * iE_r + Kd_r * dE_r;

		//Motor torque ----------------------------------------------
		tq_r = - AD_r * PIDoutput_r;

		// Clamp torque
		if (tq_r > tq_lim_r) {
			tq_r = tq_lim_r;
		}
		else if (tq_r < -tq_lim_r) {
			tq_r = -tq_lim_r;
		}
		ODrive_Set_Input_Torque(2, (float)tq_r);
		//encoder value
		ODrive_Get_Encoder_Estimates(2);
		//Target Angle ----------------------------------------------
		if (targetAngle_r > roll) {
			targetAngle_r += 1.0 * AF1_r;
		}
		if (targetAngle_r <= roll) {
			targetAngle_r -= 1.0 * AF1_r;
			}
		targetAngle_r -= AF2_r * en_vel_r;
	  }



    }




// for debug ----------------------------------------------
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum {
    IDLE,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
	EMPTY
} ControlState;

ControlState state = EMPTY;

void update_state_from_ibus() {
    if (ibus_data[1] > 1700)
        state = FORWARD;
    else if (ibus_data[1] < 1300)
        state = BACKWARD;
    else if (ibus_data[0] > 1700)
        state = RIGHT;
    else if (ibus_data[0] < 1300)
        state = LEFT;
    else if (ibus_data[2] > 1700)
        state = IDLE;
    else
    	state = EMPTY;
}

void execute_state() {
    switch (state) {
        case FORWARD:
        	AF1_w = 0;
			AF2_w = 0;
        	while (en_vel_w <= 1.0f){
				targetAngle_w -= 0.1f;
				HAL_Delay(100);
        	}
        	while (fabs(O_targetAngle_w - targetAngle_w) >= 0.7)
			{
				if (targetAngle_w <= O_targetAngle_w) {
					targetAngle_w += 0.1f;
				}
				else if (targetAngle_w > O_targetAngle_w) {
					targetAngle_w -= 0.1f;
				}
				HAL_Delay(100);
			}
        	AF1_w = O_AF1_w;
			AF2_w = O_AF2_w;

			targetAngle_w = O_targetAngle_w;
            break;
        case BACKWARD:
        	AF1_w = 0;
			AF2_w = 0;
        	while (en_vel_w >= -0.6f){
				targetAngle_w += 0.1f;
				HAL_Delay(100);
			}
        	while (fabs(O_targetAngle_w - targetAngle_w) >= 0.7)
			{
				if (targetAngle_w <= O_targetAngle_w) {
					targetAngle_w += 0.1f;
				}
				else if (targetAngle_w > O_targetAngle_w) {
					targetAngle_w -= 0.1f;
				}
				HAL_Delay(100);
			}
        	AF1_w = O_AF1_w;
			AF2_w = O_AF2_w;
			targetAngle_w = O_targetAngle_w;
            break;
        case RIGHT:
			AF1_w = 0;
			AF2_w = 0;
			targetAngle_w = O_targetAngle_w - 1.5f;
			int i = 0;
        	while (i <= 6)//(roll <= 5.0f)
        	{
        		i+=1;
				targetAngle_r += 0.2f;
				HAL_Delay(10);
			}
        	i = 0;
        	HAL_Delay(2000);
			while (fabs(O_targetAngle_w - targetAngle_w) >= 0.7)
			{
				if (targetAngle_w <= O_targetAngle_w) {
					targetAngle_w += 0.2f;
				}
				else if (targetAngle_w > O_targetAngle_w) {
					targetAngle_w -= 0.2f;
				}
				HAL_Delay(100);
			}
			targetAngle_w = O_targetAngle_w;
			while (fabs(targetAngle_r) >= 0.7f)
			{
				if (targetAngle_r <= O_targetAngle_r) {
					targetAngle_r += 0.05f;
				}
				else if (targetAngle_r > O_targetAngle_r) {
					targetAngle_r -= 0.05f;
				}
				HAL_Delay(100);
			}
			AF1_r = O_AF1_r;
			AF2_r = O_AF2_r;
			AF1_w = O_AF1_w;
			AF2_w = O_AF2_w;
			targetAngle_r = O_targetAngle_r;
            break;
        case LEFT:
        	AF1_w = 0;
			AF2_w = 0;
			targetAngle_w = O_targetAngle_w - 1.5f;
			i = 0;
			while (i <= 6)//(roll <= 5.0f)
			{
				i+=1;
				targetAngle_r -= 0.2f;
				HAL_Delay(10);
			}
			i = 0;
			HAL_Delay(2000);
			while (fabs(O_targetAngle_w - targetAngle_w) >= 0.7)
			{
				if (targetAngle_w <= O_targetAngle_w) {
					targetAngle_w += 0.2f;
				}
				else if (targetAngle_w > O_targetAngle_w) {
					targetAngle_w -= 0.2f;
				}
				HAL_Delay(100);
			}
        	targetAngle_w = O_targetAngle_w;
        	while (fabs(targetAngle_r) >= 0.7f)
        	{
				if (targetAngle_r <= O_targetAngle_r) {
					targetAngle_r += 0.05f;
				}
				else if (targetAngle_r > O_targetAngle_r) {
					targetAngle_r -= 0.05f;
				}
				HAL_Delay(100);
        	}
        	AF1_r = O_AF1_r;
			AF2_r = O_AF2_r;
			AF1_w = O_AF1_w;
			AF2_w = O_AF2_w;
			targetAngle_r = O_targetAngle_r;
            break;
        case IDLE:
        	AF1_w = 0;
			AF2_w = 0;
        	while (fabs(en_vel_w) >= 0.6f){
        		if (en_vel_w <= 0.0f){
					targetAngle_w -= 0.2f;
        		}
        		else if (en_vel_w > 0.0f){
					targetAngle_w += 0.2f;
				}
        		HAL_Delay(100);
			}
        	AF1_r = O_AF1_r;
			AF2_r = O_AF2_r;
			AF1_w = O_AF1_w;
			AF2_w = O_AF2_w;
            break;
        case EMPTY:
        	break;
    }
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  ibus_init();

  // CAN start ----------------------------------------------
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  ODrive_reboot(1);
  HAL_Delay(100);
  ODrive_reboot(2);
  HAL_Delay(100);
  ODrive_Set_Axis_State(1, 3);// FULL_CALIBRATION_SEQUENCE
  HAL_Delay(3000);

  ODrive_Set_Axis_State(2, 3);
  HAL_Delay(3000);
  ODrive_Set_Axis_State(2, 1);
  HAL_Delay(100);
  ODrive_Set_Axis_State(2, 6);
  HAL_Delay(100);

  HAL_Delay(3000);

  ODrive_Get_Axis_State(2);
  HAL_Delay(100);


  if (axis_state_2 == 1) {
      ODrive_Set_Axis_State(2, 8);
      HAL_Delay(200);
      ODrive_Get_Axis_State(2);
      printf("Axis 2 entered closed loop? State: %lu\r\n", axis_state_2);
  } else {
      printf("Axis 2 did not enter IDLE, cannot go to Closed Loop.\r\n");
  }

  /*
  	  uint32_t t_start = HAL_GetTick();
  	  while (axis_state_1 != 1 && HAL_GetTick() - t_start < 30000) {
  		  ODrive_Get_Axis_State(1); // Check state
  		  HAL_Delay(500);
  	  }

  	  if (axis_state_1 != 1) {
  		  printf("Calibration failed or timed out: axis_state_1 = %lu\r\n", axis_state_1);
  		  vTaskSuspend(NULL);
  	  }
  */
  	  // Closed loop
  ODrive_Set_Axis_State(2, 8);
  HAL_Delay(150);

  ODrive_Set_Axis_State(1, 8);
  HAL_Delay(150);

  HAL_UART_Receive_DMA(&huart1, uart1_dma_buf, DMA_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);


  HAL_Delay(150);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(150);

  HAL_TIM_Base_Start_IT(&htim1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	char msg[512];
  while (1)
  {
 //sensor data read
/*
		snprintf(msg, sizeof(msg),
			//"\n\n-----------------------------------------\r\n"
			"Wheel Target Angle: %.2f   Wheel Torque: %.2f  Encoder Wheel Velocity: %.2f\r\n"
			"RW Target Angle: %.2f   RW Torque: %.2f  Encoder RW Velocity: %.2f\r\n"
			"PIDoutput_w: %.2f PIDoutput_r: %.2f\r\n"
			"Roll: %.2f\tPitch: %.2f\r\n"
			//"-----------------------------------------\r\n\n",
			,
			targetAngle_w, tq_w, en_vel_w,
			targetAngle_r, tq_r, en_vel_r,
			PIDoutput_w, PIDoutput_r, roll, pitch);



	  	  	//printf("Roll: %.2fPitch: %.2f\r\nWheel Target Angle: %.2fRW Target Angle: %.2f\r\n",roll, pitch,targetAngle_w,targetAngle_r);
	  	  snprintf(msg, sizeof(msg),
	  			"Roll: %f, Pitch: %f, TargetAngle_w: %f, TargetAngle_r: %f\n",
				roll, pitch,targetAngle_w,targetAngle_r);

			HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, strlen(msg));
	  	  	//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			HAL_Delay(100);
*/



			ibus_read(ibus_data);
			ibus_soft_failsafe(ibus_data, 10);
			if (ibus_data[0] > 900 && ibus_data[1] > 900 &&ibus_data[2] > 900){
				update_state_from_ibus();
				execute_state();
			}
			HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/* USER CODE BEGIN 4 */

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
