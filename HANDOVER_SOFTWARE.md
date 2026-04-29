# Software & Build Handover Memo
**Project:** ECE5780 Final — ACC Smart Car  
**Date:** April 27, 2026  
**Team:** Kyuwon Hwang, Yoonhyeok Oh, Ethan Palisoc

---

## ✅ Current Status

| Item | Status |
|------|--------|
| Code written | ✅ Done |
| Build (compile) | ✅ Success |
| Flash to STM32 | ✅ Success |
| Serial monitor (USART) | ❌ Not working yet |
| Motor test | ⏳ Not tested yet |

---

## 빌드 & 플래시 방법

### 환경 (Mac에 설치된 것)
- `arm-none-eabi-gcc` v15.2 — ARM 크로스 컴파일러
- `cmake` v4.2.3
- `ninja` v1.13.2
- `st-flash` v1.8.0 — STM32 플래시 업로드 툴

### 빌드 명령어
```bash
cd /Users/kyuwonhwang/Desktop/ECE5780_Final/5780_Project_SmartCar

# 1. CMake 설정 (처음 한 번만)
cmake --preset Debug

# 2. 빌드
cmake --build --preset Debug

# 3. STM32에 플래시
st-flash write build/Debug/5780_Project_SmartCar.bin 0x8000000
```

### 빌드 결과
```
RAM:   2088B / 16KB  (12.74%)
FLASH: 8888B / 128KB (6.78%)
```

---

## 코드 구조 (Src/main.c)

### 핀맵
| STM32 핀 | 기능 |
|---------|------|
| PA2 | USART2_TX (디버그 출력) |
| PA5 | HC-SR04 Front TRIG |
| PA6 | HC-SR04 Front ECHO (전압분배 필요!) |
| PA7 | HC-SR04 Rear TRIG |
| PB0 | HC-SR04 Rear ECHO (전압분배 필요!) |
| PB4 | Motor ENABLE (PWM, TIM3_CH1) |
| PC0 | Motor INPUT1 (방향) |
| PC1 | Motor INPUT2 (방향) |

### 타이머
- TIM2: 1MHz 자유 카운터 → 초음파 펄스 측정
- TIM3: 1kHz PWM → 모터 속도

### 동작 로직
```
a) 비상정지: 장애물 ≤ 6cm → 즉시 정지
b) 앞 장애물 너무 가까움 → 후진 (PID)
c) 뒷 장애물 너무 가까움 → 전진 (PID)
d) 양쪽 안전 (dead-band 이내) → 정지
```

### 튜닝 파라미터 (main.c 맨 위)
```c
#define TARGET_DIST_CM   20   // 목표 거리 (cm)
#define DEAD_BAND_CM      2   // ±허용 오차 (cm)
#define EMERG_DIST_CM     6   // 비상정지 거리 (cm)
#define KP   12.0f            // PID P gain
#define KI    0.3f            // PID I gain
#define KD    4.0f            // PID D gain
#define PWM_MIN_DRIVE   300   // 최소 PWM (모터 dead-band 극복)
#define PWM_MAX_DRIVE   900   // 최대 PWM
```

---

## ❌ 미해결: Serial Monitor 안 됨

### 문제
ST-Link VCP(Virtual COM Port)가 Mac에서 안 잡힘.
`ls /dev/tty.*` 해도 STM32 포트 안 보임.

### 원인
ST-Link 펌웨어에 VCP 기능이 비활성화되어 있음.

### 해결 방법
STM32CubeIDE 열고:
```
Help → ST-Link Upgrade → Open in update mode → Upgrade
```
업데이트 후 USB 재연결하면 `/dev/tty.usbmodem...` 잡힘.

### 해결 후 시리얼 모니터 여는 법
```bash
screen /dev/tty.usbmodem[번호] 115200
```
출력 포맷:
```
F:20 R:20 Fe:0 Re:0 P:0 D:STP
```
- F = 앞 거리(cm), R = 뒤 거리(cm)
- Fe = 앞 에러, Re = 뒤 에러
- P = PWM 값, D = 방향(FWD/BWD/STP)

---

## 파일 위치
```
/Users/kyuwonhwang/Desktop/ECE5780_Final/5780_Project_SmartCar/
├── Src/main.c                    ← 메인 코드 (전부 여기)
├── build/Debug/
│   ├── 5780_Project_SmartCar.bin ← 플래시용 바이너리
│   └── 5780_Project_SmartCar.elf ← 디버그용
└── stm32f072xb_flash.ld          ← 링커 스크립트
```
