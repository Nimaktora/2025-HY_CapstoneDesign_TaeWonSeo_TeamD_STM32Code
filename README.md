# 2025-HY_CapstoneDesign_TaeWonSeo_TeamD_STM32Code

한양대학교 서태원 교수님 2025년 종합설계 자료입니다.
D조는 MCU로 STM32F446RE-NUCLEO를 사용하였습니다.


Control Block Diagram

![image](https://github.com/user-attachments/assets/a238eea4-d4e6-4722-a940-6ac427fc807f)


Electric Parts

![image](https://github.com/user-attachments/assets/73b2ac5b-690d-4364-973b-ce3a3f2c89c8)

IMU는 EBIMU를 사용하였습니다.

사용한 BLDC 모터 드라이버는 ODrive S1 이고 펌웨어 버전은 0.6.11 입니다.

인휠모터는 엔코더 내장인 모터뱅크 BL41105를 썼고 리액션 휠 모터는 MN6007 Kv320을 사용하고 엔코더는 AMT10E3-V를 사용하였습니다.

수정된 파일 목록

main.c

can.c

stm32f4xx_it.c

사용한 라이브러리

목화솜씨 ibus.h, ibus.c
