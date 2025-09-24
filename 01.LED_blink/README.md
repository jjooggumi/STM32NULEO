# STM32 LED Blink

GPIO를 제어하여,  
LD2 및 TEST 핀을 100ms 간격으로 점멸 동작 수행

---
## 보드 구조
<img width="583" height="360" alt="image" src="https://github.com/user-attachments/assets/7eb91a9b-112f-45ba-8528-082cfba39b32" />

---

## 📂 프로젝트 구조
- `main.c` – 메인 애플리케이션 코드
- `main.h` – 핀 정의 및 HAL 헤더
- `stm32xxxx_hal_*.c` – HAL 라이브러리 (STM32CubeMX 자동 생성)

---

## 🔧 주요 기능
- **시스템 클록 설정** – 내부 HSI 클럭 + PLL로 시스템 클록 생성
- **GPIO 초기화** – LD2, TEST 핀을 MCU에서 직접 High/Low로 제어 (Push-Pull 출력)
- **메인 루프** – LD2/TEST 핀을 0.1초 간격으로 ON/OFF

---

## 🖼 메인 루프
```c
while (1)
{
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
    HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, 0);
    HAL_Delay(100);
}
```
위 루프에서 LD2와 TEST 핀이 동시에 0.1초 간격으로 켜졌다 꺼짐

---

## 🔌 UART 설정
```c
huart2.Instance        = USART2;
huart2.Init.BaudRate   = 115200;
huart2.Init.WordLength = UART_WORDLENGTH_8B;
huart2.Init.StopBits   = UART_STOPBITS_1;
huart2.Init.Parity     = UART_PARITY_NONE;
huart2.Init.Mode       = UART_MODE_TX_RX;
huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
huart2.Init.OverSampling = UART_OVERSAMPLING_16;
HAL_UART_Init(&huart2);
```
USART2: 115200bps 설정 ( = Tera Term 속도 맞춤, 이 코드에서는 사용X)

---

## 📜 전체 코드 보기
<details>
<summary><b>main.c (펼치기/접기)</b></summary>

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

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  HAL_Init(); // HAL 라이브러리 및 SysTick 초기화, HAL 함수 동작 시 필수
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  while (1)
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
    HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, 0);
    HAL_Delay(100);
  }
}

//////////////////////////
/// 초기화 및 동작 설정///
/////////////////////////

void SystemClock_Config(void)  // 시스템 클럭 설정 (PLL, SYSCLK), MCU 실행 속도 결정
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON; // 내부 HSI 8MHz
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16; //PLL로 16배(64MHz) 시킴 
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_USART2_UART_Init(void) // UART 설정, 디버깅/데이터 통신을 위한 기본 기능 (현재 코드에서는 사용 X)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void) // GPIO 출력 모드 결정, 핀 제어 시 필요 (STM32 부팅 시 대부분 핀이 High-Z 입력 모드라 출력으로 사용하려면 초기화 필요)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, TEST_Pin|LD2_Pin, GPIO_PIN_RESET); // LD2 + TEST 핀을 출력 모드로 설정하고 초기값 LOW로 설정

  GPIO_InitStruct.Pin = TEST_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

```
</details>

---

## 🖥 실행 환경
- STM32 MCU (Nucleo 보드)
- STM32CubeIDE
- ST-LINK 디버거/프로그래머

---

## ▶ 빌드 & 실행 방법
1. STM32CubeIDE에서 프로젝트 열기
2. Build 버튼으로 컴파일
3. 보드에 펌웨어 업로드
4. LD2 및 TEST 핀이 0.1초 간격으로 점멸하는지 확인

---
## MCU 하드웨어 제어 방식 이해
### - HAL → 레지스터 → GPIO 출력
1. HAL 라이브러리 호출
```c
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
```
-> 이 한 줄이 추상화된 API

-> 실제로는 `GPIOx->BSRR`(Bit Set Reset Register)라는 레지스터에 `1`을 씀

---

2. 레지스터 Write
- HAL 함수 안쪽 코드
```c
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_PIN_ACTION(PinState));

  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16u;
  }
}
```
#### - 함수 원형과 인자
```c
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)

```
예시 그림)

<img width="500" height="100" alt="image" src="https://github.com/user-attachments/assets/f085c68d-c6dc-43b7-a18e-0d925d91af15" />


- `GPIOx`: 포트 베이스 주소(예:`GPIOA`, `GPIOB`), 내부적으로는 메모리 맵 레지스터 묶음을 가리키는 포인터
- `GPIO_Pin`: 비트마스크(예: GPIO_PIN_5), 번호가 아니라 해당 비트가 1인 값
- `PinState`: 0 또는 1

#### - 파라미터 체크
```c
assert_param(IS_GPIO_PIN(GPIO_Pin));
assert_param(IS_GPIO_PIN_ACTION(PinState));
```
- `USE_FULL_ASSERT`가 켜진 디버그 빌드에서만 유효. 잘못된 핀 마스크/상태면 assert로 중단.
- 릴리즈 빌드에선 제거되어 실행 오버헤드가 없음.

#### - 핵심: BSRR(비트 Set/Reset) 레지스터 쓰기
```c
if (PinState != GPIO_PIN_RESET) {
    GPIOx->BSRR = GPIO_Pin;
} else {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16u;
}
```
- BSRR

    : 32비트 write-only 레지스터

    : 하위 16비트 [15:0]: 해당 비트에 **1을 쓰면 그 핀을 ‘High(SET)’**로 만듦

    : 상위 16비트 [31:16]: 해당 비트에 **1을 쓰면 그 핀을 ‘Low(RESET)’**로 만듦

    : 0을 쓰면 무시함(원래 상태 유지)

    : 읽기용이 아니라 쓰기 트리거 성격임

- <<16 의미
  
    : 같은 핀 인덱스를 상위 16비트 영역으로 옮겨서 “RESET” 명령을 주기 위해

    : 예: GPIO_PIN_5(0b000…0010_0000)를 RESET하려면 1 << (5 + 16)을 BSRR에 씀

---

3. 하드웨어 반응
   - SoC 내부에서 APB 버스를 통해 GPIO IP로 데이터 전달
   - GPIO 컨트롤러 → Output Driver → 실제 핀(PAD) High/Low 전환





