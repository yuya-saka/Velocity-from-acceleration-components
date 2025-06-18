#include <Arduino_LSM9DS1.h>

float alpha = 0.95;  // ハイパスフィルタの係数（調整可能）
float beta = 0.15;    // ローパスフィルタの係数（速度用）
float accelX_prev = 0, accelY_prev = 0, accelZ_prev = 0;
float gravityX = 0, gravityY = 0, gravityZ = 0;
float velocityX = 0, velocityY = 0, velocityZ = 0; 
float prevFilteredX = 0, prevFilteredY = 0, prevFilteredZ = 0;
float fs = 119.0;  // サンプリング周波数 (Hz)
float deltaTime = 1.0 / fs; 


// ハイパスフィルタ関数（重力成分除去）
float highPassFilter(float currentValue, float &gravityValue, float alpha) {
  gravityValue = alpha * gravityValue + (1 - alpha) * currentValue;
  return currentValue - gravityValue;
}

// ローパスフィルタ関数（速度のノイズ低減）
float lowPassFilter(float currentValue, float prevValue, float beta) {
  return beta * prevValue + (1 - beta) * currentValue;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("IMUセンサーの初期化に失敗しました");
    while (1);
  }
  Serial.println("IMU センサー初期化完了");
}

void loop() {
  float accelX, accelY, accelZ;

  // 加速度データの取得
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);

    // G単位をm/s²に変換
    accelX *= 9.81;
    accelY *= 9.81;
    accelZ *= 9.81;

    // ハイパスフィルタ処理（重力成分の除去）
    float filteredX = highPassFilter(accelX, gravityX, alpha);
    float filteredY = highPassFilter(accelY, gravityY, alpha);
    float filteredZ = highPassFilter(accelZ, gravityZ, alpha);

    // ローパスフィルタでノイズ除去
    filteredX = lowPassFilter(filteredX, prevFilteredX, beta);
    filteredY = lowPassFilter(filteredY, prevFilteredY, beta);
    filteredZ = lowPassFilter(filteredZ, prevFilteredZ, beta);

    // 速度の積分（加速度から速度を計算）
    velocityX += 0.5 * (filteredX + prevFilteredX) * deltaTime;
    velocityY += 0.5 * (filteredY + prevFilteredY) * deltaTime;
    velocityZ += 0.5 * (filteredZ + prevFilteredZ) * deltaTime;

    // 静止時のドリフト補正
    float absolute_accel = sqrt(filteredX * filteredX + filteredY * filteredY + filteredZ * filteredZ);
    if (absolute_accel < 0.1) {  // 静止状態とみなすしきい値
      velocityX = 0;
      velocityY = 0;
      velocityZ = 0;
    }

    // 絶対速度を計算
    float absolute_velocity = sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ);

    Serial.print(filteredX, 2);
    Serial.print(",");
    Serial.print(filteredY, 2);
    Serial.print(",");
    Serial.print(filteredZ, 2);
    Serial.print(",");
    Serial.print(absolute_accel, 2);  //4
    Serial.print(",");
    Serial.print(velocityX, 2);
    Serial.print(",");
    Serial.print(velocityY, 2);
    Serial.print(",");
    Serial.print(velocityZ, 2);
    Serial.print(",");
    Serial.println(absolute_velocity, 2);  //8
    
    // 前回の加速度を更新
    prevFilteredX = filteredX;
    prevFilteredY = filteredY;
    prevFilteredZ = filteredZ;
  }
  //delay(1000 / fs);  // サンプリング間隔調整
}
