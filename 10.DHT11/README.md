# 🌡️ STM32 DHT11 Temperature & Humidity Sensor

TIM2로 마이크로초(us) 단위 타이머를 구현하고,  
DHT11 센서에서 온도·습도를 읽어 UART(115200bps)로 출력

---

## 📂 프로젝트 구성
- `main.c` – DHT11 통신 구현, 데이터 읽기 및 UART 전송
- `main.h` – 핀 매핑, 전역 변수 선언
- `stm32xxxx_hal_tim.c` – TIM2 초기화 및 us delay 기능
- `stm32xxxx_hal_uart.c` – UART 초기화 및 출력

---

## 🔧 주요 기능
- **DHT11 센서 통신**
  - 18ms LOW + 20~40us HIGH의 Start Signal 전송
  - 80us LOW + 80us HIGH 응답 확인
  - 5바이트 데이터(습도 정수/소수, 온도 정수/소수, 체크섬) 수신
- **데이터 검증**
  - 체크섬(`humidity + hum_decimal + temperature + temp_decimal`) 비교
  - 일치할 경우만 유효 데이터 출력
- **UART 출력**
  - `Temperature: 25℃, Humidity: 62%` 형식으로 전송
- **2초 주기 측정**
  - DHT11은 2초 이상의 간격으로 측정해야 하므로 `HAL_Delay(2000)` 적용

---

## 📜 주요 코드

### 1️⃣ DHT11 Start & Read
```c
uint8_t DHT11_Start(void) {
    DHT11_SetPinOutput();
    DHT11_SetPin(GPIO_PIN_RESET);
    HAL_Delay(20);        // 18ms LOW → 안정성을 위해 20ms
    DHT11_SetPin(GPIO_PIN_SET);
    DHT11_DelayUs(30);
    DHT11_SetPinInput();

    // 80us LOW + 80us HIGH 응답 확인
    DHT11_DelayUs(40);
    if (!(DHT11_ReadPin())) {
        DHT11_DelayUs(80);
        if (DHT11_ReadPin()) return 1;
    }
    return 0;
}
```

---

### 2️⃣ 데이터 읽기 & 체크섬 검증
```c
uint8_t DHT11_ReadData(DHT11_Data *data) {
    if (!DHT11_Start()) return 0;
    data->humidity = DHT11_ReadByte();
    data->hum_decimal = DHT11_ReadByte();
    data->temperature = DHT11_ReadByte();
    data->temp_decimal = DHT11_ReadByte();
    data->checksum = DHT11_ReadByte();

    uint8_t calc = data->humidity + data->hum_decimal +
                   data->temperature + data->temp_decimal;

    return (calc == data->checksum);
}
```

#### DHT11 데이터 프레임 구조
| 바이트 번호 | 내용 (Data)           | 설명                  | 예시 값                      |
| ------ | ------------------- | ------------------- | ------------------------- |
| **1**  | Humidity Integer    | 습도의 정수 부분 (0\~100)  | `45` → 45%                |
| **2**  | Humidity Decimal    | 습도의 소수 부분 (대부분 0)   | `0`                       |
| **3**  | Temperature Integer | 온도의 정수 부분 (섭씨 기준)   | `23` → 23°C               |
| **4**  | Temperature Decimal | 온도의 소수 부분 (대부분 0)   | `0`                       |
| **5**  | Checksum            | (1+2+3+4)의 하위 8비트 합 | `(45+0+23+0) & 0xFF = 68` |

---

### 3️⃣ 메인 루프
```c
while (1) {
    if (DHT11_ReadData(&dht11_data)) {
        sprintf(uart_buffer, "Temperature: %d℃, Humidity: %d%%\r\n",
                dht11_data.temperature, dht11_data.humidity);
    } else {
        sprintf(uart_buffer, "DHT11 Read Error!\r\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer,
                      strlen(uart_buffer), HAL_MAX_DELAY);
    HAL_Delay(2000);
}
```

## 📊 DHT11 타이밍 다이어그램

```text
MCU:   ________|¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯|_________________________
          18ms LOW (Start)      20~40us HIGH (Release)

DHT11: _____________________|¯¯¯¯|______|¯¯¯¯|_______________
                               80us L  80us H (Response)

DATA : |__|¯¯|__|¯¯|__|¯¯ ... (총 40비트)
       ↑   ↑
       |   └── HIGH 길이로 0/1 판별 (26µs=0, 70µs=1)
       └────── LOW 50µs (각 비트 시작)
```
- Start: MCU가 18ms LOW + 20~40µs HIGH 신호 전송
- Response: DHT11이 80µs LOW + 80µs HIGH 응답
- Data: 총 5바이트(습도, 온도, 체크섬) 전송
- CheckSum: 데이터 무결성 확인 (합계가 다르면 읽기 실패 처리)

## 🖥️ UART 출력
```txt
DHT11 Temperature & Humidity Sensor Test
Temperature: 24℃, Humidity: 60%
Temperature: 24℃, Humidity: 61%
DHT11 Read Error!
Temperature: 25℃, Humidity: 62%
```

## ⚙️ 실행 환경
- MCU: STM32 (예: Nucleo-F103RB)
- Sensor: DHT11 (VCC=3.3V, Data=PA0, GND)
- TIM2: 1µs 단위 카운터 (Prescaler=63 → 72MHz/64 ≈ 1.125MHz)
- UART: 115200bps (Tera Term으로 확인)

## 📜 전체 코드 보기
<details> <summary><b>main.c (펼치기/접기)</b></summary>

```c
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
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t temperature;
    uint8_t humidity;
    uint8_t temp_decimal;
    uint8_t hum_decimal;
    uint8_t checksum;
} DHT11_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DHT11_Data dht11_data;
char uart_buffer[100];  // uart_buffer 변수 선언 추가
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void DHT11_SetPinOutput(void);
void DHT11_SetPinInput(void);
void DHT11_SetPin(GPIO_PinState state);
GPIO_PinState DHT11_ReadPin(void);
void DHT11_DelayUs(uint32_t us);
uint8_t DHT11_Start(void);
uint8_t DHT11_ReadBit(void);
uint8_t DHT11_ReadByte(void);
uint8_t DHT11_ReadData(DHT11_Data *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  if (ch == '\n')
    HAL_UART_Transmit (&huart2, (uint8_t*) "\r", 1, 0xFFFF);
  HAL_UART_Transmit (&huart2, (uint8_t*) &ch, 1, 0xFFFF);

  return ch;
}

// DHT11 함수 구현
void DHT11_SetPinOutput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_SetPinInput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_SetPin(GPIO_PinState state) {
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, state);
}

GPIO_PinState DHT11_ReadPin(void) {
    return HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN);
}

void DHT11_DelayUs(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

uint8_t DHT11_Start(void) {
    uint8_t response = 0;

    // 출력 모드로 설정
    DHT11_SetPinOutput();

    // 시작 신호 전송 (18ms LOW)
    DHT11_SetPin(GPIO_PIN_RESET);
    HAL_Delay(20);  // 18ms -> 20ms로 변경 (더 안정적)

    // HIGH로 변경 후 20-40us 대기
    DHT11_SetPin(GPIO_PIN_SET);
    DHT11_DelayUs(30);

    // 입력 모드로 변경
    DHT11_SetPinInput();

    // DHT11 응답 확인 (80us LOW + 80us HIGH)
    DHT11_DelayUs(40);

    if (!(DHT11_ReadPin())) {
        DHT11_DelayUs(80);
        if (DHT11_ReadPin()) {
            response = 1;
        } else {
            response = 0;
        }
    }

    // HIGH가 끝날 때까지 대기
    while (DHT11_ReadPin());

    return response;
}

uint8_t DHT11_ReadBit(void) {
    // LOW 신호가 끝날 때까지 대기 (50us)
    while (!(DHT11_ReadPin()));

    // HIGH 신호 시작 후 30us 대기
    DHT11_DelayUs(30);

    // 여전히 HIGH면 1, LOW면 0
    if (DHT11_ReadPin()) {
        // HIGH가 끝날 때까지 대기
        while (DHT11_ReadPin());
        return 1;
    } else {
        return 0;
    }
}

uint8_t DHT11_ReadByte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte = (byte << 1) | DHT11_ReadBit();
    }
    return byte;
}

uint8_t DHT11_ReadData(DHT11_Data *data) {
    if (!DHT11_Start()) {
        return 0; // 시작 신호 실패
    }

    // 5바이트 데이터 읽기
    data->humidity = DHT11_ReadByte();
    data->hum_decimal = DHT11_ReadByte();
    data->temperature = DHT11_ReadByte();
    data->temp_decimal = DHT11_ReadByte();
    data->checksum = DHT11_ReadByte();

    // 체크섬 확인
    uint8_t calculated_checksum = data->humidity + data->hum_decimal +
                                 data->temperature + data->temp_decimal;

    if (calculated_checksum == data->checksum) {
        return 1; // 성공
    } else {
        return 0; // 체크섬 오류
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);

    // UART 초기화 메시지
    sprintf(uart_buffer, "DHT11 Temperature & Humidity Sensor Test\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    if (DHT11_ReadData(&dht11_data)) {
	      // 데이터 읽기 성공
	      sprintf(uart_buffer, "Temperature: %d℃, Humidity: %d%%\r\n",
	              dht11_data.temperature, dht11_data.humidity);
	      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	    } else {
	      // 데이터 읽기 실패
	      sprintf(uart_buffer, "DHT11 Read Error!\r\n");
	      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	    }

	    // 2초 대기 (DHT11은 최소 2초 간격으로 읽어야 함)
	    HAL_Delay(2000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
```
</details>

