#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q_current; 

// 기준점 저장 변수
Quaternion q_base;    

// 가변저항 5개 핀 설정
const int potPins[5] = {36, 39, 34, 35, 32}; 

// 캘리브레이션 범위 (필요 시 수정)
const int minRaw[5] = {0, 0, 0, 0, 0};         
const int maxRaw[5] = {4095, 4095, 4095, 4095, 4095}; 

// 영점 조절(Tare) 함수 선언
void runTareCalibration();

void setup() {
  Serial.begin(115200);
  
  Wire.begin(21, 22);
  Wire.setClock(400000); 

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 연결 실패");
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    // [개선] 센서 칩 내부 필터가 안정화될 때까지 5초간 대기합니다.
    Serial.println(">> 센서 웜업 및 안정화 중 (5초)... 가만히 두세요.");
    delay(5000); 
    
    // 안정화가 끝난 후 첫 영점 조절 실행
    runTareCalibration();

  } else {
    Serial.print("DMP 초기화 실패: ");
    Serial.println(devStatus);
    while (1);
  }
}

void loop() {
  if (!dmpReady) return;

  // ★ 시리얼 모니터에 't' 또는 'T' 입력 시 언제든 실시간 영점 재조절 가능
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 't' || cmd == 'T') {
      runTareCalibration();
    }
  }

  // 1. 가변저항 5개 값 읽기
  float c[5];
  for (int i = 0; i < 5; i++) {
    c[i] = readCurl(potPins[i], minRaw[i], maxRaw[i]);
  }

  // 2. MPU6050 데이터 가져오기
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q_current, fifoBuffer);

    // 3. 상대 쿼터니언 연산 (현재 각도 - 기준 각도)
    float q1_w = q_base.w;
    float q1_x = -q_base.x; // Conjugate
    float q1_y = -q_base.y;
    float q1_z = -q_base.z;

    float q2_w = q_current.w;
    float q2_x = q_current.x;
    float q2_y = q_current.y;
    float q2_z = q_current.z;

    float qw = q1_w * q2_w - q1_x * q2_x - q1_y * q2_y - q1_z * q2_z;
    float qx = q1_w * q2_x + q1_x * q2_w + q1_y * q2_z - q1_z * q2_y;
    float qy = q1_w * q2_y - q1_x * q2_z + q1_y * q2_w + q1_z * q2_x;
    float qz = q1_w * q2_z + q1_x * q2_y - q1_y * q2_x + q1_z * q2_w;

    // 4. Unity 송신 규격 출력
    for (int i = 0; i < 5; i++) {
      Serial.print(c[i], 4);
      Serial.print(",");
    }
    Serial.print(qw, 4); Serial.print(",");
    Serial.print(qx, 4); Serial.print(",");
    Serial.print(qy, 4); Serial.print(",");
    Serial.println(qz, 4);
  }

  delay(16); 
}

// 영점 조절 기능을 담당하는 독립 함수
void runTareCalibration() {
  Serial.println(">> 기준점(0도) 세팅 중... 1.5초간 센서를 움직이지 마세요.");
  mpu.resetFIFO();
  delay(50);
  
  unsigned long startTime = millis();
  float sumW = 0, sumX = 0, sumY = 0, sumZ = 0;
  int sampleCount = 0;

  while (millis() - startTime < 1500) { // 1.5초간 데이터 수집
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion tempQ;
      mpu.dmpGetQuaternion(&tempQ, fifoBuffer);
      
      sumW += tempQ.w;
      sumX += tempQ.x;
      sumY += tempQ.y;
      sumZ += tempQ.z;
      sampleCount++;
    }
  }

  if (sampleCount > 0) {
    q_base.w = sumW / sampleCount;
    q_base.x = sumX / sampleCount;
    q_base.y = sumY / sampleCount;
    q_base.z = sumZ / sampleCount;

    float mag = sqrt(q_base.w*q_base.w + q_base.x*q_base.x + q_base.y*q_base.y + q_base.z*q_base.z);
    q_base.w /= mag; q_base.x /= mag; q_base.y /= mag; q_base.z /= mag;
    
    Serial.println(">> 영점 조절 완료! 표준 상태(1, 0, 0, 0)로 리셋되었습니다.");
  }
  mpu.resetFIFO();
}

float readCurl(int pin, int minVal, int maxVal) {
  int raw = analogRead(pin);
  if (maxVal == minVal) return 0.0;
  float normalized = (float)(raw - minVal) / (maxVal - minVal);
  if (normalized < 0.0) normalized = 0.0;
  if (normalized > 1.0) normalized = 1.0;
  return normalized;
}
