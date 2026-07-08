# ESP32 기반 스마트 글러브 (Unity 연동 시스템)

ESP32 DevKitC V4 개발 보드와 MPU6050 IMU 센서, 그리고 5개의 가변저항(Potentiometer)을 활용하여 사용자의 손가락 움직임과 손목의 회전 데이터를 실시간으로 측정하고 Unity 3D 환경으로 송신하는 하드웨어 펌웨어 시스템입니다.

---

# 시스템 사양 및 통신 규격
- 주요 부품: ESP32 DevKitC V4, MPU6050 (6축 가속도/자이로 센서), 가변저항 5개
- 통신 방식: USB Serial (가상 COM 포트)
- 보레이트 (Baud Rate, 통신 속도): 115200 bps
- 데이터 갱신 주기: 약 60Hz (delay 16ms)
- 출력 포맷 (CSV 형태의 단일 라인 문자열):
  ```text
  c0,c1,c2,c3,c4,qw,qx,qy,qz\n
