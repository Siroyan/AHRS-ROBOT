#include <EasyGPS.h>
#include <MPU9250.h>

/* モーター出力ピン */
#define AIN1 D2
#define AIN2 D3
#define BIN1 D6
#define BIN2 D9
/* 回転モード */
#define  CW 0 // ClockWise:時計回り
#define CCW 1 // CounterClockWise:反時計回り
/* 角度 */
float targetAngle = 0;
float currentYawAngle = 0;
/* 緯度経度 */
float targetIdo = 35.001122;
float targetKeido = 139.001122;
float currentIdo = 35.001122;
float currentKeido = 139.001122;
/* 残りの距離 */
float distance = 0.00;
/* PWMのduty比 */
float rotationDuty = 0;
float distanceDuty = 0;
float duty = 0;
/* 回転モード */
int rotationMode = CW;

MPU9250 mpu;
EasyGPS gps;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.setup();
  gps.setup();
  
  mpu.calibrateAccelGyro();
  Serial.println("Accl,Gyro Calibration Finish.");
  mpu.calibrateMag();
  Serial.println("Mag Calibration Finish.");
  mpu.printCalibration();
}

void loop(){
  static uint32_t prevMs = millis();
  if((millis() - prevMs) > 10){

    /* 現在のyawの角度を取得 */
    mpu.update();
    currentYawAngle = mpu.getYaw();

    /* 現在の緯度と経度を取得 */
    gps.update();

    /* 目標の角度と残りの距離を取得 */
    targetAngle = gps.getAzimuth(targetIdo, targetKeido);
    distance = gps.getDistance(targetIdo, targetKeido);
    
    /* 右回りか左回りか */
    if((targetAngle - currentYawAngle) > 0){
      rotationMode = CW;
    }else{
      rotationMode = CCW;
    }

    /* pidでduty比を決定 */
    rotationPidControl();
    distancePidControl();

    /* duty比の最終決定 */
    duty = rotationDuty + distanceDuty;
    
    /* いったん初期化 */
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
    
    /* pidで決めたduty比と回転方向によってモーター出力を決定 */
    if(rotationMode == CW){
      analogWrite(AIN2, duty);
      analogWrite(BIN1, duty);
    }else{
      analogWrite(AIN1, duty);
      analogWrite(BIN2, duty);
    }
    
    prevMs = millis();
  }
}

void rotationPidControl(){
  const float Kp = 1, Ki = 0, Kd = 0;
  float dt, preTime;
  float P, I, D, preP = 0;
  dt = (millis() - preTime) / 1000;
  preTime = millis();
  P  = abs(targetAngle - currentYawAngle);
  I += P * dt;
  D  = (P - preP) / dt;
  preP = P;

  rotationDuty = Kp * P + Ki * I + Kd * D;
}

void distancePidControl(){
  const float Kp = 1, Ki = 0, Kd = 0;
  float dt, preTime;
  float P, I, D, preP = 0;
  dt = (millis() - preTime) / 1000;
  preTime = millis();
  P  = distance;
  I += P * dt;
  D  = (P - preP) / dt;
  preP = P;

  distanceDuty = Kp * P + Ki * I + Kd * D;
}
