# STM32NULEO
- 참고 도서
<img width="200" height="320" alt="image" src="https://github.com/user-attachments/assets/ff6c153d-7370-4092-aaac-8196dcd8724a" />

: 김남수,이진형, 『STM32CubeIDE를 이용한 STM32 따라하기』, 출판지: 북랩

## 목차
1. ADC_TemperatureSensor
2. EXTI
3. HC-SR04
4. LED_blink
5. Motor_TEST
6. TIM_TimBase
7. USART_printf


## 1. ADC_TemperatureSensor
- ADC(Analog-to-Digital Convertor) 예제
  - 목적: 디바이스 내부 온도센서 이용하여 ADC 모드와 특성 이해
  - Pinout & Configuration 설정
    - RCC(Reset and Clock Controller): 내부 클럭만 사용
    - ADC1: Temperature Sensor Channel로 선택
    - Parameter Setting
      - Continuous Conversion Mode = Enable, 온도 센서 신호 => 온도 측정 값 (연속으로 AD 변환 동작)
      - Sampling Time = 13.5 cycles
  - Clock Configuration 설정
    - 
## 2. EXTI

## 3. HC-SR04

## 4. LED_blink

## 5. Motor_TEST

## 6. TIM_TimBase

## 7. USART_printf
