# Hardware Handover Memo
**Project:** ECE5780 Final — ACC Smart Car  
**Date:** April 27, 2026  
**Team:** Kyuwon Hwang, Yoonhyeok Oh, Ethan Palisoc

---

## 보유 부품

| 부품 | 수량 | 상태 |
|------|------|------|
| STM32F072 Discovery Board | 1 | ✅ 정상 |
| Custom Motor Driver PCB (ECE5780 HW1) | 2 | ✅ 조립 완료 |
| HC-SR04 Ultrasonic Sensor | 2 | ✅ 보유 |
| 2WD 차체 + DC 기어모터 (Pololu) | 1세트 | ✅ 보유 |
| DC 전원 공급기 | 1 | ✅ 사용 중 |

---

## 현재 하드웨어 구성

### 전원
- 배터리 제거함
- **DC 전원 공급기**로 모터 전원 공급 중
- DC 전원 → Custom PCB 배럴잭 연결
- 전압: **6V** 권장 (모터 스펙)

### 모터 드라이버 구성
- Custom PCB **1개만** 사용 (앞뒤 직선 운동만 하므로)
- 두 모터를 **병렬**로 PCB 출력에 연결
  ```
  PCB MTR_RED → Motor A 빨간선 + Motor B 빨간선
  PCB MTR_BLK → Motor A 검정선 + Motor B 검정선
  ```
- Custom PCB 2번째는 미사용 (보관)

---

## 전체 배선도

### STM32 → Custom PCB
| STM32 핀 | Custom PCB 핀 | 설명 |
|---------|-------------|------|
| PB4 | ENABLE | PWM 속도 제어 |
| PC0 | INPUT1 | 방향 제어 |
| PC1 | INPUT2 | 방향 제어 |
| 5V | 5V | L298N 로직 전원 |
| 3V3 | 3V3 | LM75A 온도센서 전원 |
| GND | GND | 공통 GND ⚠️ 필수 |

### STM32 → HC-SR04 Front (앞 센서)
| STM32 핀 | HC-SR04 핀 | 설명 |
|---------|-----------|------|
| PA5 | TRIG | 트리거 펄스 출력 |
| PA6 | ECHO | 에코 수신 ⚠️ 전압분배 필요 |
| 5V | VCC | 센서 전원 |
| GND | GND | 공통 GND |

### STM32 → HC-SR04 Rear (뒤 센서)
| STM32 핀 | HC-SR04 핀 | 설명 |
|---------|-----------|------|
| PA7 | TRIG | 트리거 펄스 출력 |
| PB0 | ECHO | 에코 수신 ⚠️ 전압분배 필요 |
| 5V | VCC | 센서 전원 |
| GND | GND | 공통 GND |

### 전원
| 연결 | 설명 |
|------|------|
| DC 전원(+) → PCB 배럴잭 | 모터 구동 전원 (6V) |
| DC 전원(-) → GND 공통 | STM32 GND와 반드시 공통 연결 |
| STM32 USB → Mac | STM32 전원 + 플래시 |

---

## ⚠️ ECHO 핀 전압 분배 (필수!)

HC-SR04 ECHO = 5V 출력 → STM32는 3.3V만 허용
전압 분배 안 하면 STM32 핀 손상됨!

```
ECHO(5V) ──[1kΩ]──┬── STM32 핀 (PA6 or PB0)
                  [1kΩ]
                   │
                  GND

결과: 5V × 1/2 = 2.5V ✅ 안전
```

앞 센서(PA6)와 뒤 센서(PB0) 둘 다 적용!

---

## ❌ 미테스트 항목

- [ ] 모터 실제 구동 테스트
- [ ] 모터 방향 확인 (반대면 INPUT1↔INPUT2 바꾸기)
- [ ] HC-SR04 거리 측정 정확도 확인
- [ ] PID 게인 튜닝 (KP=12, KI=0.3, KD=4.0)
- [ ] 전체 ACC 동작 통합 테스트

---

## 다음 할 일

1. **Serial Monitor 활성화** → ST-Link 펌웨어 업데이트
2. **센서 테스트** → 시리얼로 F:XX R:XX 값 확인
3. **모터 방향 테스트** → 전진/후진 방향 맞는지 확인
4. **PID 튜닝** → KP부터 조정, KI/KD는 나중에
5. **통합 테스트** → 장애물 앞에서 실제 ACC 동작 확인

---

## 주의사항

- DC 전원과 STM32 GND 반드시 공통 연결할 것
- 전원 켜기 전 배선 두 번 확인할 것
- 모터 막히는 상황 오래 지속되면 L298N 과열 주의
- ECHO 핀 전압 분배 저항 꼭 확인할 것
