#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// DMP 변수
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q_current; // 실시간 읽어오는 쿼터니언

// ★ 기준점 저장을 위한 변수
Quaternion q_base;    // 2초 동안 평균 낸 기준 쿼터니언

// 가변저항 5개 핀 설정 (엄지, 검지, 중지, 약지, 소지)
const int potPins[5] = {36, 39, 34, 35, 32}; 

// 캘리브레이션 (필요 시 수정)
const int minRaw[5] = {0, 0, 0, 0, 0};         
const int maxRaw[5] = {4095, 4095, 4095, 4095, 4095}; 

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
    
    // -------------------------------------------------------------
    // [영점 조절 알고리즘] 리셋 후 2초간 데이터 수집 및 평균 계산
    // -------------------------------------------------------------
    Serial.println(">> 2초간 기준점(0도)을 세팅합니다. 센서를 움직이지 마세요...");
    
    unsigned long startTime = millis();
    float sumW = 0, sumX = 0, sumY = 0, sumZ = 0;
    int sampleCount = 0;

    while (millis() - startTime < 2000) { // 2000ms(2초) 동안 루프
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        Quaternion tempQ;
        mpu.dmpGetQuaternion(&tempQ, fifoBuffer);
        
        sumW += tempQ.w;
        sumX += tempQ.x;
        sumY += tempQ.y;
        sumZ += tempQ.z;
        sampleCount++;
      }
      delay(10);
    }

    if (sampleCount > 0) {
      // 누적된 쿼터니언의 평균값 계산
      q_base.w = sumW / sampleCount;
      q_base.x = sumX / sampleCount;
      q_base.y = sumY / sampleCount;
      q_base.z = sumZ / sampleCount;

      // 크기를 1로 만드는 정규화(Normalize) 수행
      float mag = sqrt(q_base.w * q_base.w + q_base.x * q_base.x + q_base.y * q_base.y + q_base.z * q_base.z);
      q_base.w /= mag;
      q_base.x /= mag;
      q_base.y /= mag;
      q_base.z /= mag;
      
      Serial.println(">> 기준점 세팅 완료! 측정을 시작합니다.");
    } else {
      Serial.println(">> 경고: 센서 데이터를 받지 못해 기본값으로 세팅합니다.");
      q_base = Quaternion(1, 0, 0, 0);
    }
    // -------------------------------------------------------------

  } else {
    Serial.print("DMP 초기화 실패: ");
    Serial.println(devStatus);
    while (1);
  }
}

void loop() {
  if (!dmpReady) return;

  // 1. 가변저항 5개 값 읽기
  float c[5];
  for (int i = 0; i < 5; i++) {
    c[i] = readCurl(potPins[i], minRaw[i], maxRaw[i]);
  }

  // 2. MPU6050 데이터 가져오기
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q_current, fifoBuffer);

    // 3. 상대 쿼터니언 계산 (현재 각도에서 기준 각도를 빼는 연산)
    // 수학적으로 q_relative = (q_base의 켤레 쿼터니언) * q_current 입니다.
    float q1_w = q_base.w;
    float q1_x = -q_base.x; // Conjugate (켤레)
    float q1_y = -q_base.y;
    float q1_z = -q_base.z;

    float q2_w = q_current.w;
    float q2_x = q_current.x;
    float q2_y = q_current.y;
    float q2_z = q_current.z;

    // 쿼터니언 곱셈 공식 적용
    float qw = q1_w * q2_w - q1_x * q2_x - q1_y * q2_y - q1_z * q2_z;
    float qx = q1_w * q2_x + q1_x * q2_w + q1_y * q2_z - q1_z * q2_y;
    float qy = q1_w * q2_y - q1_x * q2_z + q1_y * q2_w + q1_z * q2_x;
    float qz = q1_w * q2_z + q1_x * q2_y - q1_y * q2_x + q1_z * q2_w;

    // 4. Unity 송신 규격 출력 (c0,c1,c2,c3,c4,qw,qx,qy,qz)
    for (int i = 0; i < 5; i++) {
      Serial.print(c[i], 4);
      Serial.print(",");
    }
    Serial.print(qw, 4); Serial.print(",");
    Serial.print(qx, 4); Serial.print(",");
    Serial.print(qy, 4); Serial.print(",");
    Serial.println(qz, 4);
  }

  delay(16); // 약 60Hz
}

float readCurl(int pin, int minVal, int maxVal) {
  int raw = analogRead(pin);
  if (maxVal == minVal) return 0.0;
  
  float normalized = (float)(raw - minVal) / (maxVal - minVal);
  
  if (normalized < 0.0) normalized = 0.0;
  if (normalized > 1.0) normalized = 1.0;
  
  return normalized;
}