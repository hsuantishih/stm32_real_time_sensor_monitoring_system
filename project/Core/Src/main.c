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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "myprintf.h"
#include "string.h"
#include "File_Handling.h"
#include "semphr.h"
#include "queue.h"
#include "stdarg.h"
#include "adxl.h"
#include "math.h"
#include "../../ECUAL/I2C_LCD/I2C_LCD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	DISPLAY_OFF,
  MODE_SELECTION,
  REALTIME_DISPLAY,
  HISTORY_VIEW
} SystemState;

typedef enum {
	NO_PRESS,
	SINGLE_PRESS,
	LONG_PRESS,
	DOUBLE_PRESS
} eButtonEvent;

typedef struct {
	char message[128];
} RecordMessage;

typedef struct {
  float ax;
  float ay;
  float az;
  uint32_t current_step_count;
} PedometerData;

typedef struct {
  uint32_t index;
  uint32_t steps;
  uint32_t duration_ms;
} HistoryRecord;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MyI2C_LCD I2C_LCD_1
#define mylcdprintf(idx, col, row, str)        mylcdprintf_ext(idx, col, row, str, true)
#define mylcdprintf_noclear(idx, col, row, str)  mylcdprintf_ext(idx, col, row, str, false)
#define MAX_HISTORY_RECORDS 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t last_step_time = 0;
static uint32_t step_count = 0;
static uint32_t record_index = 0;
static HistoryRecord historyData[MAX_HISTORY_RECORDS];
static uint8_t total_history_count = 0;
static uint8_t current_history_index = 0;
TaskHandle_t buttonTaskHandle;
SystemState currentState = DISPLAY_OFF;
SemaphoreHandle_t fatfsMutex;
QueueHandle_t recordQueue;
QueueHandle_t PedometerDataQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PedometerTask(void *pvParameters) {
  float ax, ay, az;
  float acc_mag;
  float prev_acc_mag = 0;
  PedometerData pedData;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ);

  while (1) {
    if (currentState == REALTIME_DISPLAY) {
      Adxl_Read_Acceleration(&ax, &ay, &az);
  
      acc_mag = sqrt(ax * ax + ay * ay + az * az);
      acc_mag = acc_mag - 1.0f; // Remove gravity
  
      if (prev_acc_mag < STEP_THRESHOLD && acc_mag >= STEP_THRESHOLD) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
  
        if (current_time - last_step_time > STEP_DEBOUNCE_MS) {
          step_count++;
  
          myprintf("Step detected! Global step count: %lu\r\n", step_count);
          last_step_time = current_time;
        }
      }
  
      pedData.ax = ax;
      pedData.ay = ay;
      pedData.az = az;
      pedData.current_step_count = step_count;
  
      if (PedometerDataQueue != NULL) {
        xQueueSend(PedometerDataQueue, &pedData, pdMS_TO_TICKS(100));
      }
  
      prev_acc_mag = acc_mag;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void RecordOperation(const char *format, ...){
	RecordMessage msg;
	va_list args;
	va_start(args, format);
	vsnprintf(msg.message, sizeof(msg.message), format, args);
	va_end(args);
	if (xQueueSend(recordQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS){
		myprintf("Failed to send record message!\r\n");
	}
}

void PrintRecordFile(void) {
	static FIL recordFile;
	static FRESULT fr;
	UINT br;
	char buffer[128];
	xSemaphoreTake(fatfsMutex, portMAX_DELAY);
	fr = f_open(&recordFile, "record.csv", FA_READ);
	if (fr != FR_OK) {
		myprintf("Failed to open record.csv (Error: %d)\r\n", fr);
		xSemaphoreGive(fatfsMutex);
		return;
	}
	myprintf("Record file contents:\r\n");
	do {
		fr = f_read(&recordFile, buffer, sizeof(buffer) - 1, &br);
		if (fr != FR_OK) {
			myprintf("Error reading record.csv (Error: %d)\r\n", fr);
			break;
		}
		buffer[br] = '\0';
		myprintf("%s", buffer);
	} while (br == sizeof(buffer) - 1);

	f_close(&recordFile);
	xSemaphoreGive(fatfsMutex);
}

void RecordTask(void *pvParameters) {
	static FIL recordFile;
	static FRESULT fr;
	UINT bw;
	RecordMessage recordMsg;
	bool fileOpened = false;

	xSemaphoreTake(fatfsMutex, portMAX_DELAY);
	fr = f_open(&recordFile, "record.csv", FA_CREATE_ALWAYS | FA_WRITE);
	if (fr == FR_OK) {
		f_close(&recordFile);
		myprintf("Record file cleared.\r\n");
	} else {
		myprintf("Failed to clear record file, error = %d\r\n", fr);
	}
	xSemaphoreGive(fatfsMutex);

	for(;;){
		if (xQueueReceive(recordQueue, &recordMsg, portMAX_DELAY) == pdPASS) {
			xSemaphoreTake(fatfsMutex, portMAX_DELAY);
			if(!fileOpened) {
				fr = f_open(&recordFile, "record.csv", FA_OPEN_APPEND | FA_WRITE);
				if (fr != FR_OK){
					myprintf("RecordTask: Failed to open record.csv, error = %d\r\n", fr);
					xSemaphoreGive(fatfsMutex);
					continue;
				}
				fileOpened = true;
			}

			fr = f_write(&recordFile, recordMsg.message, strlen(recordMsg.message), &bw);
			if (fr == FR_OK){
				f_sync(&recordFile);
				myprintf("RecordTask: Wrote record entry.\r\n");

				f_close(&recordFile);
				fileOpened = false;
				xSemaphoreGive(fatfsMutex);
				PrintRecordFile();
			} else {
				myprintf("RecordTask: f_write error: %d\r\n", fr);
				xSemaphoreGive(fatfsMutex);
			}
		}
	}
}

void LoadHistoryFromSD(void) {
    static FIL recordfile;
    FRESULT fr;
    char line[64];
    
    total_history_count = 0;

    xSemaphoreTake(fatfsMutex, portMAX_DELAY);
    fr = f_open(&recordfile, "record.csv", FA_READ);
    if (fr == FR_OK) {
        while (f_gets(line, sizeof(line), &recordfile) != NULL) {
            if (total_history_count >= MAX_HISTORY_RECORDS) {
                break;
            }
            
            uint32_t idx, steps, dur;
            // Parse String: "Index,Steps,Duration"
            if (sscanf(line, "%lu,%lu,%lu", &idx, &steps, &dur) == 3) {
                historyData[total_history_count].index = idx;
                historyData[total_history_count].steps = steps;
                historyData[total_history_count].duration_ms = dur;
                total_history_count++;
            }
        }
        f_close(&recordfile);
        myprintf("History loaded: %d records.\r\n", total_history_count);
    } else {
        myprintf("LoadHistory: Failed to open record.csv (Error %d)\r\n", fr);
    }
    xSemaphoreGive(fatfsMutex);
}

bool buttonState() {
    static const uint32_t DEBOUNCE_MILLIS = 50;
    static uint32_t last_debounce_time = 0;
    static bool stable_state = false;
    static bool last_read_state = false;

    bool current_read = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);

    if (current_read != last_read_state) {
        last_debounce_time = HAL_GetTick();
    }

    if ((HAL_GetTick() - last_debounce_time) > DEBOUNCE_MILLIS) {
        if (current_read != stable_state) {
            stable_state = current_read;
        }
    }

    last_read_state = current_read;
    return stable_state;
}

eButtonEvent getButtonEvent() {
    static const uint32_t DOUBLE_GAP_MILLIS_MAX = 250;
    static const uint32_t LONG_MILLIS_MIN = 800;

    static uint32_t button_down_ts = 0;
    static uint32_t button_up_ts = 0;
    static bool double_pending = false;
    static bool long_press_fired = false;
    static bool prev_button_state = false;

    eButtonEvent event = NO_PRESS;
    uint32_t now = HAL_GetTick();
    bool current_state = buttonState();

    if (current_state && !prev_button_state) {
        button_down_ts = now;
        long_press_fired = false;
    }
    else if (!current_state && prev_button_state) {
        if (!long_press_fired) {
            if (double_pending && (now - button_up_ts <= DOUBLE_GAP_MILLIS_MAX)) {
                event = DOUBLE_PRESS;
                double_pending = false;
            } else {
                double_pending = true;
                button_up_ts = now;
            }
        }
    }
    else if (current_state && prev_button_state) {
        if (!long_press_fired && (now - button_down_ts >= LONG_MILLIS_MIN)) {
            event = LONG_PRESS;
            long_press_fired = true;  
            double_pending = false;
        }
    }
    else if (!current_state && !prev_button_state) {
        if (double_pending && (now - button_up_ts > DOUBLE_GAP_MILLIS_MAX)) {
            event = SINGLE_PRESS;
            double_pending = false;
        }
    }

    prev_button_state = current_state;
    return event;
}

void ButtonTask(void *pvParameters) {
    static SystemState prevState = (SystemState)-1; 
    int mode_selection_index = 0; // For navigating mode selection
    int prev_mode_selection_index = -1; // To track changes in mode selection for LCD updates
    PedometerData receivedPedData;

    uint8_t is_recording = 0; // 0 = Not Recording, 1 = Recording
    uint32_t record_start_time_ms = 0;
    uint32_t record_start_steps = 0; 

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));

        // State Change
        if (currentState != prevState) {
            switch (currentState) {
                case DISPLAY_OFF:
                    mylcdprintf(MyI2C_LCD, 0, 0, "Welcome to Pedometer");
                    mylcdprintf_noclear(MyI2C_LCD, 0, 2, "Press to Start");
                    break;
                    
                case MODE_SELECTION:
                    mode_selection_index = 0;
                    prev_mode_selection_index = -1;
                    
                    mylcdprintf(MyI2C_LCD, 0, 0, "Press to Select Mode");
                    mylcdprintf_noclear(MyI2C_LCD, 2, 1, "Real-time Display   ");
                    mylcdprintf_noclear(MyI2C_LCD, 2, 2, "History View        "); 
                    break;
                    
                case REALTIME_DISPLAY:
                    I2C_LCD_Clear(MyI2C_LCD);
                    is_recording = 0; // Reset recording state when entering real-time display
                    break;
                    
                case HISTORY_VIEW:     
                    I2C_LCD_Clear(MyI2C_LCD);
                    LoadHistoryFromSD();
                    current_history_index = 0;
                    break;
                    
                default: break;
            }
            prevState = currentState;
        }

        // State-specific Updates
        switch (currentState) {
            case MODE_SELECTION:
                if (mode_selection_index != prev_mode_selection_index) {
                    if (mode_selection_index == 0) {

                        mylcdprintf_noclear(MyI2C_LCD, 0, 2, " ");
                        mylcdprintf_noclear(MyI2C_LCD, 0, 1, "> Real-time Display   ");
                    } else {
                        mylcdprintf_noclear(MyI2C_LCD, 0, 1, " ");
                        mylcdprintf_noclear(MyI2C_LCD, 0, 2, "> History View        ");
                    }
                    prev_mode_selection_index = mode_selection_index;
                }
                break;
                
            case REALTIME_DISPLAY:
                if (PedometerDataQueue != NULL && xQueueReceive(PedometerDataQueue, &receivedPedData, 0) == pdPASS) {
                    char display_str_0[20], display_str_1[20];
                    char ax_str[16], ay_str[16], az_str[16];

                    if (is_recording) {
                        uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        uint32_t duration_sec = (current_time_ms - record_start_time_ms) / 1000;
                        uint32_t session_steps = receivedPedData.current_step_count - record_start_steps;

                        snprintf(display_str_0, sizeof(display_str_0), "Time: %lu s        ", duration_sec);
                        snprintf(display_str_1, sizeof(display_str_1), "Rec Steps: %lu    ", session_steps);
                    } else {
                      snprintf(display_str_0, sizeof(display_str_0), "Ready to record    ");
                      snprintf(display_str_1, sizeof(display_str_1), "Total Steps:%lu     ", receivedPedData.current_step_count);
                    }

                    snprintf(ax_str, sizeof(ax_str), "X:%.2f", receivedPedData.ax);
                    snprintf(ay_str, sizeof(ay_str), "Y:%.2f", receivedPedData.ay);
                    snprintf(az_str, sizeof(az_str), "Z:%.2f", receivedPedData.az);
                    
                    mylcdprintf_noclear(MyI2C_LCD, 0, 0, display_str_0);
                    mylcdprintf_noclear(MyI2C_LCD, 0, 1, display_str_1);
                    mylcdprintf_noclear(MyI2C_LCD, 0, 2, ax_str);
                    mylcdprintf_noclear(MyI2C_LCD, 10, 2, ay_str);
                    mylcdprintf_noclear(MyI2C_LCD, 0, 3, az_str);
                }
                break;
            
            case HISTORY_VIEW:
                if (total_history_count == 0) {
                    mylcdprintf_noclear(MyI2C_LCD, 0, 0, "No History Found  ");
                    mylcdprintf_noclear(MyI2C_LCD, 0, 1, "                  ");
                    mylcdprintf_noclear(MyI2C_LCD, 0, 2, "                  ");
                    mylcdprintf_noclear(MyI2C_LCD, 0, 3, "                  ");
                } else {
                    char str0[20], str1[20], str2[20];
                    // Get current history record based on current_history_index
                    HistoryRecord cur_rec = historyData[current_history_index];
                    
                    // Display: "Rec: (1/2)"
                    snprintf(str0, sizeof(str0), "Rec: (%d/%d)  ", current_history_index + 1, total_history_count);
                    snprintf(str1, sizeof(str1), "Steps: %lu       ", cur_rec.steps);
                    snprintf(str2, sizeof(str2), "Time: %lu s      ", cur_rec.duration_ms / 1000);
                    
                    mylcdprintf_noclear(MyI2C_LCD, 0, 0, str0);
                    mylcdprintf_noclear(MyI2C_LCD, 0, 1, str1);
                    mylcdprintf_noclear(MyI2C_LCD, 0, 2, str2);
                    mylcdprintf_noclear(MyI2C_LCD, 0, 3, "                  ");
                }
                break;
            default:
                break;
        }

        // Trigger Button Event
        eButtonEvent event = getButtonEvent();
        if (event != NO_PRESS) {
            switch (currentState) {
                case DISPLAY_OFF:
                    if (event == DOUBLE_PRESS) {
                        myprintf("Display Off: Double Press Detected.\r\n");
                        currentState = MODE_SELECTION;
                    }
                    break;
                    
                case MODE_SELECTION:
                    if (event == SINGLE_PRESS) {
                        myprintf("MODE_SELECTION: Single Press Detected.\r\n");
                        // Change current selected mode
                        mode_selection_index = (mode_selection_index + 1) % 2;
                    } else if (event == DOUBLE_PRESS) {
                        myprintf("MODE_SELECTION: Double Press Detected.\r\n");
                        // Enter Current Selected Mode
                        if (mode_selection_index == 0) {
                            currentState = REALTIME_DISPLAY;
                        } else {
                            currentState = HISTORY_VIEW;
                        }
                    } else if (event == LONG_PRESS) {
                        myprintf("MODE_SELECTION: Long Press Detected. Exiting.\r\n");
                        currentState = DISPLAY_OFF;
                    }
                    break;
                    
                case REALTIME_DISPLAY:
                    if (event == LONG_PRESS) {
                        myprintf("REALTIME_DISPLAY: Long Press Detected.\r\n");
                        currentState = MODE_SELECTION;
                    } else if (event == SINGLE_PRESS) {
                        myprintf("REALTIME_DISPLAY: Single Press Detected.\r\n");
                        // First Time: Start recording steps
                        if (is_recording == 0) {
                          is_recording = 1;
                            record_start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            record_start_steps = receivedPedData.current_step_count;
                            myprintf("REALTIME_DISPLAY: Start Recording...\r\n");
                        } else {
                          // Second Time logic: Stop recording
                          is_recording = 0;
                          uint32_t end_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                          uint32_t duration_ms = end_time_ms - record_start_time_ms;
                          uint32_t final_session_steps = receivedPedData.current_step_count - record_start_steps;
                          RecordOperation("%lu,%lu,%lu\r\n", record_index++, final_session_steps, duration_ms);

                          myprintf("REALTIME_DISPLAY: Stopped Recording.\r\n");
                          myprintf(">>> Session Result - Duration: %lu ms, Steps Taken: %lu <<<\r\n", duration_ms, final_session_steps);
                        }
                    }
                    break;
                    
                case HISTORY_VIEW:
                    if (event == LONG_PRESS) {
                        myprintf("HISTORY_VIEW: Long Press Detected.\r\n");
                        currentState = MODE_SELECTION;
                    } else if (event == SINGLE_PRESS) {
                        myprintf("HISTORY_VIEW: Single Press Detected. Switching Record.\r\n");
                        // Switch between different history records
                        if (total_history_count > 0) {
                            current_history_index = (current_history_index + 1) % total_history_count;
                        }
                    }
                    break;

                default:
                    break;
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  Adxl_Init();
  I2C_LCD_Init(MyI2C_LCD);
  
  if (Mount_SD() != FR_OK) {
    I2C_LCD_Clear(MyI2C_LCD);
    mylcdprintf(MyI2C_LCD, 0, 0, "SD Mount Failed!");

    while (1)
    {
      HAL_Delay(1000);
    }
  }


  fatfsMutex = xSemaphoreCreateMutex();
  if (fatfsMutex == NULL){
  	myprintf("Failed to create fatfsMutex!\r\n");
  }
  recordQueue = xQueueCreate(10, sizeof(RecordMessage));
  if (recordQueue == NULL){
  	myprintf("Failed to create recordQueue!\r\n");
  }


  PedometerDataQueue = xQueueCreate(10, sizeof(PedometerData));
  if (PedometerDataQueue == NULL){
  	myprintf("Failed to create PedometerDataQueue!\r\n");
  }

  xTaskCreate(RecordTask, "RecordTask", 512, NULL, 3, NULL);
  xTaskCreate(ButtonTask, "ButtonTask", 512, NULL, 1, &buttonTaskHandle);
  xTaskCreate(PedometerTask, "PedometerTask", 256, NULL, 2, NULL);
  vTaskStartScheduler();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
